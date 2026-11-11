// Copyright 2026 EngineAI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <boost/thread/recursive_mutex.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_msgs/srv/set_entity_state.hpp>
#include <gazebo_ros/node.hpp>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

namespace
{

const std::vector<std::string> kJointNames = {
  "J00_HIP_PITCH_L", "J01_HIP_ROLL_L", "J02_HIP_YAW_L",
  "J03_KNEE_PITCH_L", "J04_ANKLE_PITCH_L", "J05_ANKLE_ROLL_L",
  "J06_HIP_PITCH_R", "J07_HIP_ROLL_R", "J08_HIP_YAW_R",
  "J09_KNEE_PITCH_R", "J10_ANKLE_PITCH_R", "J11_ANKLE_ROLL_R",
  "J12_WAIST_YAW",
  "J13_SHOULDER_PITCH_L", "J14_SHOULDER_ROLL_L",
  "J15_SHOULDER_YAW_L", "J16_ELBOW_PITCH_L", "J17_ELBOW_YAW_L",
  "J18_SHOULDER_PITCH_R", "J19_SHOULDER_ROLL_R",
  "J20_SHOULDER_YAW_R", "J21_ELBOW_PITCH_R", "J22_ELBOW_YAW_R",
  "J23_HEAD_YAW",
};

const std::vector<double> kNominalPositions = {
  -0.06, 0.0, 0.0, 0.12, -0.06, 0.0,
  -0.06, 0.0, 0.0, 0.12, -0.06, 0.0,
  0.0,
  0.0, 0.15, 0.0, -0.25, 0.0,
  0.0, -0.15, 0.0, -0.25, 0.0,
  0.0,
};

}  // namespace

class Pm01StandResetPlugin : public gazebo::ModelPlugin
{
public:
  ~Pm01StandResetPlugin() override { startup_update_.reset(); }

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    world_ = model->GetWorld();
    node_ = gazebo_ros::Node::Get(sdf);
    startup_pose_ = model_->WorldPose();
    hold_startup_ = sdf->HasElement("hold_until_stand_reset") &&
      sdf->Get<bool>("hold_until_stand_reset");
    hold_publisher_ = node_->create_publisher<std_msgs::msg::Bool>(
      "/pm01/sim_startup_held", rclcpp::QoS(1).transient_local());
    PublishHoldState();
    if (hold_startup_) {
      startup_update_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&Pm01StandResetPlugin::HoldStartup, this));
      RCLCPP_INFO(node_->get_logger(), "Holding startup pose until the standing reset");
    }
    service_ = node_->create_service<gazebo_msgs::srv::SetEntityState>(
      "/pm01/snap_stand",
      std::bind(
        &Pm01StandResetPlugin::OnSnap, this,
        std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(
      node_->get_logger(),
      "PM01 stand reset plugin ready on /pm01/snap_stand");
  }

private:
  void PublishHoldState()
  {
    std_msgs::msg::Bool message;
    message.data = hold_startup_.load();
    hold_publisher_->publish(message);
  }

  void HoldStartup()
  {
    // Physics must advance to activate ros2_control. Hold the robot while
    // those updates and the initial-pose/policy handoff are taking place.
    if (!hold_startup_) {
      return;
    }
    SnapJoints();
    model_->SetWorldPose(startup_pose_);
    model_->SetLinearVel(ignition::math::Vector3d::Zero);
    model_->SetAngularVel(ignition::math::Vector3d::Zero);
    for (const auto & link : model_->GetLinks()) {
      link->SetLinearVel(ignition::math::Vector3d::Zero);
      link->SetAngularVel(ignition::math::Vector3d::Zero);
    }
  }

  void OnSnap(
    const std::shared_ptr<gazebo_msgs::srv::SetEntityState::Request> request,
    std::shared_ptr<gazebo_msgs::srv::SetEntityState::Response> response)
  {
    if (!model_ || !world_ || !world_->Physics()) {
      response->success = false;
      RCLCPP_ERROR(
        node_->get_logger(), "Gazebo model or physics engine is unavailable");
      return;
    }

    boost::recursive_mutex::scoped_lock lock(
      *world_->Physics()->GetPhysicsUpdateMutex());
    if (!SnapJoints()) {
      response->success = false;
      return;
    }

    const auto & position = request->state.pose.position;
    const auto & orientation = request->state.pose.orientation;
    model_->SetWorldPose(
      ignition::math::Pose3d(
        ignition::math::Vector3d(position.x, position.y, position.z),
        ignition::math::Quaterniond(
          orientation.w, orientation.x, orientation.y, orientation.z)));
    model_->SetLinearVel(ignition::math::Vector3d::Zero);
    model_->SetAngularVel(ignition::math::Vector3d::Zero);
    for (const auto & link : model_->GetLinks()) {
      link->SetLinearVel(ignition::math::Vector3d::Zero);
      link->SetAngularVel(ignition::math::Vector3d::Zero);
    }

    // The reset coordinator has paused physics, reset the running policy and
    // queued nominal targets. Release only after the complete snap succeeded.
    hold_startup_ = false;
    PublishHoldState();
    response->success = true;
  }

  bool SnapJoints()
  {
    bool success = true;
    for (std::size_t index = 0; index < kJointNames.size(); ++index) {
      auto joint = model_->GetJoint(kJointNames[index]);
      if (!joint) {
        RCLCPP_ERROR(
          node_->get_logger(), "Reset joint not found: %s",
          kJointNames[index].c_str());
        success = false;
        continue;
      }
      joint->SetPosition(0, kNominalPositions[index], true);
      joint->SetVelocity(0, 0.0);
    }
    return success;
  }

  std::atomic<bool> hold_startup_{false};
  ignition::math::Pose3d startup_pose_;
  gazebo::event::ConnectionPtr startup_update_;
  gazebo::physics::ModelPtr model_;
  gazebo::physics::WorldPtr world_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Service<gazebo_msgs::srv::SetEntityState>::SharedPtr service_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hold_publisher_;
};

GZ_REGISTER_MODEL_PLUGIN(Pm01StandResetPlugin)

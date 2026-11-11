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

#include "simulator/pm01_pd_math.hpp"

#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <controller_interface/helpers.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace simulator
{

class Pm01SdkPdController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override
  {
    try {
      auto_declare<std::vector<std::string>>("joints", {});
      auto_declare<std::vector<double>>("kp", {});
      auto_declare<std::vector<double>>("kd", {});
      auto_declare<std::vector<double>>("default_positions", {});
      auto_declare<std::vector<double>>("effort_limits", {});
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_node()->get_logger(), "%s", error.what());
      return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration configuration;
    configuration.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : joints_) {
      configuration.names.push_back(
        joint + "/" + hardware_interface::HW_IF_EFFORT);
    }
    return configuration;
  }

  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration configuration;
    configuration.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : joints_) {
      configuration.names.push_back(
        joint + "/" + hardware_interface::HW_IF_POSITION);
      configuration.names.push_back(
        joint + "/" + hardware_interface::HW_IF_VELOCITY);
    }
    return configuration;
  }

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State &) override
  {
    joints_ = get_node()->get_parameter("joints").as_string_array();
    kp_ = get_node()->get_parameter("kp").as_double_array();
    kd_ = get_node()->get_parameter("kd").as_double_array();
    default_positions_ =
      get_node()->get_parameter("default_positions").as_double_array();
    effort_limits_ =
      get_node()->get_parameter("effort_limits").as_double_array();

    const std::size_t count = joints_.size();
    if (
      count == 0 || kp_.size() != count || kd_.size() != count ||
      default_positions_.size() != count ||
      effort_limits_.size() != count)
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "joints, kp, kd, default_positions and effort_limits must have "
        "the same non-zero length");
      return controller_interface::CallbackReturn::ERROR;
    }
    for (std::size_t index = 0; index < count; ++index) {
      if (
        !std::isfinite(kp_[index]) || !std::isfinite(kd_[index]) ||
        !std::isfinite(default_positions_[index]) ||
        !std::isfinite(effort_limits_[index]) ||
        kp_[index] < 0.0 || kd_[index] < 0.0 ||
        effort_limits_[index] <= 0.0)
      {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "Invalid PD parameter at joint %s", joints_[index].c_str());
        return controller_interface::CallbackReturn::ERROR;
      }
    }

    target_buffer_.writeFromNonRT(
      std::make_shared<std::vector<double>>(default_positions_));
    command_subscription_ =
      get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/commands", rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr message) {
        if (message->data.size() != joints_.size()) {
          RCLCPP_ERROR_THROTTLE(
            get_node()->get_logger(), *get_node()->get_clock(), 2000,
            "Ignoring target with %zu values; expected %zu",
            message->data.size(), joints_.size());
          return;
        }
        for (const double value : message->data) {
          if (!std::isfinite(value)) {
            RCLCPP_ERROR_THROTTLE(
              get_node()->get_logger(), *get_node()->get_clock(), 2000,
              "Ignoring non-finite joint target");
            return;
          }
        }
        target_buffer_.writeFromNonRT(
          std::make_shared<std::vector<double>>(message->data));
      });
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &) override
  {
    ordered_effort_interfaces_.clear();
    ordered_position_interfaces_.clear();
    ordered_velocity_interfaces_.clear();
    const bool effort_ok = controller_interface::get_ordered_interfaces(
      command_interfaces_, joints_, hardware_interface::HW_IF_EFFORT,
      ordered_effort_interfaces_);
    const bool position_ok = controller_interface::get_ordered_interfaces(
      state_interfaces_, joints_, hardware_interface::HW_IF_POSITION,
      ordered_position_interfaces_);
    const bool velocity_ok = controller_interface::get_ordered_interfaces(
      state_interfaces_, joints_, hardware_interface::HW_IF_VELOCITY,
      ordered_velocity_interfaces_);
    if (!effort_ok || !position_ok || !velocity_ok) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Failed to order PM01 effort, position or velocity interfaces");
      return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &) override
  {
    for (auto & command : ordered_effort_interfaces_) {
      command.get().set_value(0.0);
    }
    ordered_effort_interfaces_.clear();
    ordered_position_interfaces_.clear();
    ordered_velocity_interfaces_.clear();
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update(
    const rclcpp::Time &, const rclcpp::Duration &) override
  {
    const auto * target_pointer = target_buffer_.readFromRT();
    if (!target_pointer || !(*target_pointer)) {
      return controller_interface::return_type::ERROR;
    }
    const auto & targets = *(*target_pointer);
    for (std::size_t index = 0; index < joints_.size(); ++index) {
      const double position =
        ordered_position_interfaces_[index].get().get_value();
      const double velocity =
        ordered_velocity_interfaces_[index].get().get_value();
      ordered_effort_interfaces_[index].get().set_value(
        compute_pd_effort(
          targets[index], position, velocity,
          kp_[index], kd_[index], effort_limits_[index]));
    }
    return controller_interface::return_type::OK;
  }

private:
  std::vector<std::string> joints_;
  std::vector<double> kp_;
  std::vector<double> kd_;
  std::vector<double> default_positions_;
  std::vector<double> effort_limits_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<std::vector<double>>>
  target_buffer_;
  std::vector<
    std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
  ordered_effort_interfaces_;
  std::vector<
    std::reference_wrapper<hardware_interface::LoanedStateInterface>>
  ordered_position_interfaces_;
  std::vector<
    std::reference_wrapper<hardware_interface::LoanedStateInterface>>
  ordered_velocity_interfaces_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
    command_subscription_;
};

}  // namespace simulator

PLUGINLIB_EXPORT_CLASS(
  simulator::Pm01SdkPdController,
  controller_interface::ControllerInterface)

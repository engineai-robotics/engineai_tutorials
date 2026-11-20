#include <fstream>
#include <set>
#include <sstream>
#include <iomanip>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
int main(int argc,char**argv){
 if(argc < 2)return 2;const std::string output=argv[1];
 rclcpp::init(argc,argv);auto node=std::make_shared<rclcpp::Node>("refactor_tf_owner_probe");
 std::set<std::string> ids;
 auto sub=node->create_subscription<tf2_msgs::msg::TFMessage>("/tf",100,
  [&](tf2_msgs::msg::TFMessage::ConstSharedPtr msg,const rclcpp::MessageInfo &info){
   for(const auto&t:msg->transforms)if(t.header.frame_id=="map"&&t.child_frame_id=="odom"){
    std::ostringstream id;for(auto v:info.get_rmw_message_info().publisher_gid.data)id<<std::hex<<std::setw(2)<<std::setfill('0')<<int(v);
    ids.insert(id.str());std::ofstream out(output);for(const auto&v:ids)out<<v<<"\n";
   }
  });
 rclcpp::spin(node);rclcpp::shutdown();
}

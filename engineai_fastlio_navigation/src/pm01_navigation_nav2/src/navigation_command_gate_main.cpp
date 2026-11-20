#include "pm01_navigation_nav2/navigation_command_gate.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(pm01_navigation::make_navigation_command_gate());
  rclcpp::shutdown();
  return 0;
}

#include "velocity_control.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityControl>());
  rclcpp::shutdown();
  return 0;
}
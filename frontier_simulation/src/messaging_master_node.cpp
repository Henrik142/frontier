#include "messaging_master.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MessagingMaster>());
  rclcpp::shutdown();
  return 0;
}
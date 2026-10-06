#include "astar_test.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<AstarTest>());
  rclcpp::shutdown();
  return 0;
}
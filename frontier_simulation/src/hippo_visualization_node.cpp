#include "hippo_visualization.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HippoVisualization>());
  rclcpp::shutdown();
  return 0;
}
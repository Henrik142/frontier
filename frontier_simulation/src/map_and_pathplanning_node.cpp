#include "map_updater.hpp"
#include "path_planning_manager.hpp"
#include "planning_maps.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // The MapUpdater holds the "real" map that it operates on during a point cloud callback.
  auto map_updater = std::make_shared<MapUpdater>();

  Map::InitMapParams map_params;
  map_updater->getMapParams(map_params);

  auto planning_maps = std::make_shared<PlanningMaps>(map_params);

  map_updater->initPlanningMaps(planning_maps);

  auto path_planning_manager = std::make_shared<PathPlanningManager>(
    planning_maps, map_updater->getMapReadyFlag());

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(map_updater);
  executor.add_node(path_planning_manager);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}

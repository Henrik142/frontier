#include "map_updater.hpp"
#include "path_planning_manager.hpp"
#include "trajectory_planner.hpp"
#include "planning_maps.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // The MapUpdater holds the "real" map that it operates on during a point cloud callback.
  auto map_updater = std::make_shared<MapUpdater>();

  Map::InitMapParams map_params;
  map_updater->getMapParams(map_params);

  // Map consumers:
  // MapUpdater (publishMap)
  // PathPlanningManager (pathPlanningCallback)
  // TrajectoryPlanner (planTrajectory)
  size_t num_map_consumers = 3;
  auto planning_maps = std::make_shared<PlanningMaps>(map_params, num_map_consumers);

  map_updater->initPlanningMaps(planning_maps);

  auto path_planning_manager = std::make_shared<PathPlanningManager>(
    planning_maps, map_updater->getMapReadyFlag());

  auto trajectory_planner = std::make_shared<TrajectoryPlanner>(
    planning_maps);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(map_updater);
  executor.add_node(path_planning_manager);
  executor.add_node(trajectory_planner);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}

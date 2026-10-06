#include "map_updater.hpp"
#include "path_planning_manager.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto map_updater = std::make_shared<MapUpdater>();
  // PathPlanningManager gets the same Map instance so both nodes stay in sync without serialization.
  auto path_planning_manager = std::make_shared<PathPlanningManager>(
      map_updater->getMap());

  // Single-threaded executor keeps callback execution serialized (no locking needed on the shared map).
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(map_updater);
  executor.add_node(path_planning_manager);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}

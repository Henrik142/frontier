#include "trajectory_planner.hpp"

TrajectoryPlanner::TrajectoryPlanner() {
}

TrajectoryPlanner::~TrajectoryPlanner() {
}

void TrajectoryPlanner::initTrajectoryPlanner(const std::shared_ptr<Map> & map,
                      const Astar::InitAstarParams & astar_params) {
    map_ = map;

    astar_.reset(new Astar);
    astar_->init(astar_params, map_);
}
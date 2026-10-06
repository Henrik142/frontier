#pragma once

#include <Eigen/Eigen>

#include "map.hpp"
#include "astar.hpp"

class TrajectoryPlanner {
public:
    TrajectoryPlanner();
    ~TrajectoryPlanner();

    void initTrajectoryPlanner(const std::shared_ptr<Map> & map,
                      const Astar::InitAstarParams & astar_params);

    // Utils
    std::unique_ptr<Astar> astar_;

private:
    // Data (owned by MapUpdater, shared here)
    std::shared_ptr<Map> map_;

};
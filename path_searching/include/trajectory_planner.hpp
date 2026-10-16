#pragma once

#include <Eigen/Eigen>

#include "map.hpp"
#include "astar.hpp"

class TrajectoryPlanner {
public:
    TrajectoryPlanner();
    ~TrajectoryPlanner();

    void initTrajectoryPlanner(const std::shared_ptr<const Map> & map,
                      const Astar::InitAstarParams & astar_params);
    void setMap(const std::shared_ptr<const Map>& map);

    // Utils
    std::unique_ptr<Astar> astar_;

private:
    // Data
    std::shared_ptr<const Map> map_;
};
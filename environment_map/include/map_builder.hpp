#pragma once

#include <Eigen/Eigen>
#include "map.hpp"

class MapBuilder {
public:
    explicit MapBuilder(Map* map);
    void buildTestScene();
    void setBox(const Eigen::Vector2i& min_id, const Eigen::Vector2i& max_id, int occupancy);
    void setBox(const Eigen::Vector2d& min_pos, const Eigen::Vector2d& max_pos, int occupancy);

private:
    Map* map_;
};
#include <Eigen/Eigen>
#include <stdexcept>

#include "map_builder.hpp"

MapBuilder::MapBuilder(Map* map) : map_(map) {
    if (map_ == nullptr) {
        throw std::invalid_argument("map pointer cannot be null");
    }
}

void MapBuilder::buildTestScene() {
    Eigen::Vector2i map_min, map_max;
    map_->getMapBox(map_min, map_max);
    setBox(map_min, map_max, Map::FREE);

    Eigen::Vector2d min = Eigen::Vector2d(0, 0.5);
    Eigen::Vector2d max = Eigen::Vector2d(2.5, 0.7);
    setBox(min, max, Map::OCCUPIED);

    min = Eigen::Vector2d(1.2, 0.9);
    max = Eigen::Vector2d(1.3, 2.5);
    setBox(min, max, Map::OCCUPIED);

    min = Eigen::Vector2d(-0.8, -2.0);
    max = Eigen::Vector2d(-0.7, 1.0);
    setBox(min, max, Map::OCCUPIED);

    min = Eigen::Vector2d(0.2, -1.8);
    max = Eigen::Vector2d(1.0, -1.0);
    setBox(min, max, Map::OCCUPIED);

    min = Eigen::Vector2d(0.25, -0.4);
    max = Eigen::Vector2d(0.8, 0.0);
    setBox(min, max, Map::OCCUPIED);

    min = Eigen::Vector2d(1.4, -1.3);
    max = Eigen::Vector2d(1.9, 0.0);
    setBox(min, max, Map::OCCUPIED);

    min = Eigen::Vector2d(-1.8, 1.5);
    max = Eigen::Vector2d(0.2, 1.8);
    setBox(min, max, Map::OCCUPIED);

    min = Eigen::Vector2d(-2.5, 0.6);
    max = Eigen::Vector2d(-1.35, 0.7);
    setBox(min, max, Map::OCCUPIED);

    min = Eigen::Vector2d(-2.0, 0.1);
    max = Eigen::Vector2d(-0.8, 0.2);
    setBox(min, max, Map::OCCUPIED);

    min = Eigen::Vector2d(-2.0, -0.4);
    max = Eigen::Vector2d(-0.8, -0.3);
    setBox(min, max, Map::OCCUPIED);

    min = Eigen::Vector2d(-2.5, -1.1);
    max = Eigen::Vector2d(-1.0, -1.0);
    setBox(min, max, Map::OCCUPIED);

    min = Eigen::Vector2d(-2.0, -2.0);
    max = Eigen::Vector2d(-0.8, -1.8);
    setBox(min, max, Map::OCCUPIED);


}

void MapBuilder::setBox(const Eigen::Vector2i& min_id, const Eigen::Vector2i& max_id, int occupancy) {
    for (int i = min_id(0); i <= max_id(0); ++i) {
        for (int j = min_id(1); j <= max_id(1); ++j) {
            Eigen::Vector2i id(i, j);
            map_->setOccupancy(id, occupancy);
        }
    }
}

void MapBuilder::setBox(const Eigen::Vector2d& min_pos, const Eigen::Vector2d& max_pos, int occupancy) {
    Eigen::Vector2i min_id, max_id;
    map_->posToIndex(min_pos, min_id);
    map_->posToIndex(max_pos, max_id);
    setBox(min_id, max_id, occupancy);
}
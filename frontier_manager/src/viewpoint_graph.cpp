#include "viewpoint_graph.hpp"

// Static data
double ViewNode::vm_;
double ViewNode::yd_;
double ViewNode::w_dir_;
std::shared_ptr<Astar> ViewNode::astar_;
std::shared_ptr<RayCaster> ViewNode::caster_;
std::shared_ptr<Map> ViewNode::map_;

// Graph node for viewpoints planning
ViewNode::ViewNode(const Eigen::Vector2d& p, const double& y) {
    pos_ = p;
    yaw_ = y;
    parent_ = nullptr;
    vel_.setZero();  // vel is zero by default, should be set explicitly
}

double ViewNode::costTo(const ViewNode::Ptr& node) {
    std::vector<Eigen::Vector2d> path;
    double c = ViewNode::computeCost(pos_, node->pos_, yaw_, node->yaw_, vel_, path);
    return c;
}

double ViewNode::searchPath(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, std::vector<Eigen::Vector2d>& path) {
    // Try connect two points with straight line
    bool safe = true;
    Eigen::Vector2i idx;
    caster_->input(p1, p2);
    while (caster_->nextId(idx)) {
        if (!(map_->getInflatedOccupancy(idx) == Map::FREE) || !map_->isInMap(idx)) {
            safe = false;
            break;
        }
    }
    if (safe) {
        path = { p1, p2 };
        return (p1 - p2).norm();
    }

    /*
    // Search a path using decreasing resolution
    // the smallest resolution determines the smallest gap that
    // can be navigated by the path search
    std::vector<double> res = {0.3, 0.2, 0.1 };
    for (int k = 0; k < res.size(); ++k) {
        astar_->reset();
        astar_->setResolution(res[k]);
        if (astar_->search(p1, p2) == Astar::REACH_END) {
            path = astar_->getPath();
            return astar_->pathLength(path);
        }
    }
    */

    astar_->reset();
    if (astar_->search(p1, p2) == Astar::REACH_END) {
        path = astar_->getPath();
        return astar_->pathLength(path);
    }

    // Return very high path length if path cannot be found
    path = { p1, p2 };
    return 1000;
}

double ViewNode::computeCost(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const double& y1, const double& y2,
                             const Eigen::Vector2d& v1, std::vector<Eigen::Vector2d>& path) {
    // Cost of position change
    // Path cost = Path length / maximum velocity
    double pos_cost = ViewNode::searchPath(p1, p2, path) / vm_;

    /* Motion consistency cost
    This is only added for the connection cost between the robot's own viewpoint
    and any cluster, since when calculating the cost between a pair of clusters,
    the velocity is zero.
    It penalizes large changes in flight direction to prevent back-and-forth
    maneuvers in successive planning steps.
    */
    Eigen::Vector2d delta = p2 - p1;
    if (v1.norm() > 1e-3 && delta.norm() > 1e-9) {
        Eigen::Vector2d vdir = v1.normalized();
        Eigen::Vector2d dir = delta.normalized();

        double cosine = std::clamp(vdir.dot(dir), -1.0, 1.0);
        double angle = std::acos(cosine);

        pos_cost += w_dir_ * angle;
    }

    // Cost of yaw change
    // yaw cost = difference in yaw angles / maximum yaw rate
    /*
    double diff = fabs(y2 - y1);
    diff = std::min(diff, 2 * M_PI - diff);
    double yaw_cost = diff / yd_;
    */
    // yaw cost = (difference in yaw angles between starting yaw and first path segment +
    //             difference in yaw angles between last path segment and end yaw ) 
    //             / maximum yaw rate
    

    double yaw_cost = 0.0;

    /*
    Eigen::Vector2d dir_start = (path.empty() ? Eigen::Vector2d::Zero() : path.front() - p1);
    Eigen::Vector2d dir_end = (path.empty() ? Eigen::Vector2d::Zero() : p2 - path.back());

    if (dir_start.norm() > 1e-9) {
        double cosine_start = std::clamp(std::cos(y1) * dir_start.x() + std::sin(y1) * dir_start.y(), -1.0, 1.0);
        double angle_start = std::acos(cosine_start);
        yaw_cost += angle_start / yd_;
    }
    if (dir_end.norm() > 1e-9) {
        double cosine_end = std::clamp(std::cos(y2) * dir_end.x() + std::sin(y2) * dir_end.y(), -1.0, 1.0);
        double angle_end = std::acos(cosine_end);
        yaw_cost += angle_end / yd_;
    }
    */


    // Total cost is the maximum of position cost and yaw cost
    return std::max(pos_cost, yaw_cost);
}
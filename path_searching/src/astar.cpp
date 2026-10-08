#include "astar.hpp"

Astar::Astar() {
}

Astar::~Astar() {
  for (int i = 0; i < allocate_num_; i++)
    delete path_node_pool_[i];
}

void Astar::init(const InitAstarParams& params, const std::shared_ptr<Map>& map) {
    // TODO: make parameters configurable through ROS2 parameters

    // Spatial grid spacing used by A*
    // Independent of the map’s underlying resolution, but it should
    // generally be equal to or larger than the map resolution.
    resolution_ = params.resolution;
    this->inv_resolution_ = 1.0 / resolution_;

    // Controls the weight of the heuristic in
    // f_score = g_score + lambda_heu_ * heuristic
    // Larger values make A* prioritize speed and explore fewer nodes
    lambda_heu_ = params.lambda_heu;

    // Maximum time allowed for one search, in seconds
    // When the limit is reached, the search returns NO_PATH,
    // even though a path might exist
    max_search_time_ = params.max_search_time;

    // Number of Node objects preallocated in the search pool
    // If the pool is exhausted, the search fails
    allocate_num_ = params.allocate_num;
    
    /* ---------- map params ---------- */
    this->map_ = map;    
    getOrigin(origin_);
    getSize(map_size_);

    tie_breaker_ = 1.0 + 1.0 / 1000;

    // Preallocate the pool of Node objects for the search
    path_node_pool_.resize(allocate_num_);
    for (int i = 0; i < allocate_num_; i++) {
        path_node_pool_[i] = new Node;
    }
    use_node_num_ = 0;
    iter_num_ = 0;
    early_terminate_cost_ = 0.0;
}

void Astar::setResolution(const double& res) {
    resolution_ = res;
    this->inv_resolution_ = 1.0 / resolution_;
}

int Astar::search(const Eigen::Vector2d& start_pt, const Eigen::Vector2d& end_pt) {
    NodePtr cur_node = path_node_pool_[0];
    cur_node->parent = NULL;
    cur_node->position = start_pt;
    posToIndex(start_pt, cur_node->index);

    cur_node->g_score = 0.0;
    cur_node->f_score = lambda_heu_ * getDiagHeu(cur_node->position, end_pt);

    Eigen::Vector2i end_index;
    posToIndex(end_pt, end_index);

    open_set_.push(cur_node);
    open_set_map_.insert(std::make_pair(cur_node->index, cur_node));
    use_node_num_ += 1;

    const auto t1 = std::chrono::steady_clock::now();

    /* ---------- search loop ---------- */
    while (!open_set_.empty()) {
        cur_node = open_set_.top();
        bool reach_end = abs(cur_node->index(0) - end_index(0)) <= 1 &&
            abs(cur_node->index(1) - end_index(1)) <= 1 &&
            checkLineOfSight(cur_node->position, end_pt);
        if (reach_end) {
            backtrack(cur_node, end_pt);
            if (path_nodes_[0] != start_pt) {
                path_nodes_.insert(path_nodes_.begin(), start_pt);
            }
            return REACH_END;
        }

        // Early termination if time up
        if (std::chrono::duration<double>(std::chrono::steady_clock::now() - t1).count() > max_search_time_) {
            early_terminate_cost_ = cur_node->g_score + getDiagHeu(cur_node->position, end_pt);
            return NO_PATH_TIMEOUT;
        }

        open_set_.pop();
        open_set_map_.erase(cur_node->index);
        close_set_map_.insert(std::make_pair(cur_node->index, 1));
        iter_num_ += 1;

        Eigen::Vector2d cur_pos = cur_node->position;

        for (int dx_index = -1; dx_index <= 1; ++dx_index) {
            for (int dy_index = -1; dy_index <= 1; ++dy_index) {

                if (dx_index == 0 && dy_index == 0) {
                    continue;
                }

                Eigen::Vector2i step_index(dx_index, dy_index);
                Eigen::Vector2i nbr_idx = cur_node->index + step_index;

                if (!map_->isInMap(nbr_idx)) {
                    continue;
                }

                Eigen::Vector2d nbr_pos = origin_ +
                    (nbr_idx.cast<double>().array() + 0.5).matrix() * resolution_;


                // Check safety
                if (!isInMap(nbr_pos)) continue;
                if (!(getInflatedOccupancy(nbr_pos) == Map::FREE))
                    continue;
                
                // Check line of sight, except for the start point
                if (!(cur_pos == start_pt)) {
                    bool safe = checkLineOfSight(cur_pos, nbr_pos);
                    if (!safe) continue;
                }

                // Check not in close set
                if (close_set_map_.find(nbr_idx) != close_set_map_.end()) continue;

                NodePtr neighbor;
                double step_cost = step_index.cast<double>().norm() * resolution_;
                double tmp_g_score = cur_node->g_score + step_cost;
                auto node_iter = open_set_map_.find(nbr_idx);
                if (node_iter == open_set_map_.end()) {
                    neighbor = path_node_pool_[use_node_num_];
                    use_node_num_ += 1;
                    if (use_node_num_ == allocate_num_) {
                        return NO_PATH_MEMORY;
                    }
                    neighbor->index = nbr_idx;
                    neighbor->position = nbr_pos;
                } else if (tmp_g_score < node_iter->second->g_score) {
                    neighbor = node_iter->second;
                } else
                    continue;

                neighbor->parent = cur_node;
                neighbor->g_score = tmp_g_score;
                neighbor->f_score = tmp_g_score + lambda_heu_ * getDiagHeu(nbr_pos, end_pt);
                open_set_.push(neighbor);
                open_set_map_[nbr_idx] = neighbor;
            }
        }
            
    }

    return NO_PATH_INVALID;
}

double Astar::getEarlyTerminateCost() {
    return early_terminate_cost_;
}

void Astar::reset() {
    open_set_map_.clear();
    close_set_map_.clear();
    path_nodes_.clear();

    std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> empty_queue;
    open_set_.swap(empty_queue);
    for (int i = 0; i < use_node_num_; i++) {
        path_node_pool_[i]->parent = NULL;
    }
    use_node_num_ = 0;
    iter_num_ = 0;
}

double Astar::pathLength(const std::vector<Eigen::Vector2d>& path) {
    double length = 0.0;
    if (path.size() < 2) return length;
    for (int i = 0; i < path.size() - 1; ++i)
        length += (path[i + 1] - path[i]).norm();
    return length;
}

// could be replaced with raycaster
bool Astar::checkLineOfSight(const Eigen::Vector2d& start, const Eigen::Vector2d& end) {
    // Check line of sight for safety
    Eigen::Vector2d dir = end - start;
    double len = dir.norm();
    dir.normalize();
    double los_step = getMapResolution();
    for (double l = los_step; l < len; l += los_step) {
        Eigen::Vector2d ckpt = start + l * dir;
        if (!(getInflatedOccupancy(ckpt) == Map::FREE)) {
            return false;
        }
    }
    return true;
}

void Astar::backtrack(const NodePtr& end_node, const Eigen::Vector2d& end) {
    path_nodes_.clear();
    std::vector<Eigen::Vector2d> reversed_path;

    reversed_path.push_back(end);
    NodePtr cur_node = end_node;
    while (cur_node != NULL) {
        if ((reversed_path.back() - cur_node->position).norm() > 1e-9) {
            reversed_path.push_back(cur_node->position);
        }
        cur_node = cur_node->parent;
    }

    if (reversed_path.size() > 1 &&
        (reversed_path.front() - reversed_path.back()).norm() < 1e-9) {
        reversed_path.pop_back();
    }

    std::reverse(reversed_path.begin(), reversed_path.end());
    path_nodes_ = std::move(reversed_path);
}

std::vector<Eigen::Vector2d> Astar::getPath() {
    return path_nodes_;
}

double Astar::getDiagHeu(const Eigen::Vector2d& x1, const Eigen::Vector2d& x2) {
    double dx = fabs(x1(0) - x2(0));
    double dy = fabs(x1(1) - x2(1));
    double diag = std::min(dx, dy);
    double straight = std::max(dx, dy) - diag;
    double h = sqrt(2.0) * diag + straight;
    return tie_breaker_ * h;
}

double Astar::getManhHeu(const Eigen::Vector2d& x1, const Eigen::Vector2d& x2) {
    double dx = fabs(x1(0) - x2(0));
    double dy = fabs(x1(1) - x2(1));
    return tie_breaker_ * (dx + dy);
}

double Astar::getEuclHeu(const Eigen::Vector2d& x1, const Eigen::Vector2d& x2) {
    return tie_breaker_ * (x2 - x1).norm();
}

std::vector<Eigen::Vector2d> Astar::getVisited() {
    std::vector<Eigen::Vector2d> visited;
    for (int i = 0; i < use_node_num_; ++i)
        visited.push_back(path_node_pool_[i]->position);
    return visited;
}

void Astar::posToIndex(const Eigen::Vector2d& pt, Eigen::Vector2i& idx) {
    idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();
}

/* Wrappers of Map class */

double Astar::getMapResolution() {
    return map_->getResolution();
}

void Astar::getOrigin(Eigen::Vector2d& origin) {
    origin[0] = map_->getOriginX();
    origin[1] = map_->getOriginY();
}

void Astar::getSize(Eigen::Vector2d& size) {
    size[0] = map_->getSizeX();
    size[1] = map_->getSizeY();
}

bool Astar::isInMap(const Eigen::Vector2d& pos) {
    return map_->isInMap(pos);
}

int Astar::getInflatedOccupancy(const Eigen::Vector2d& pos) {
    return map_->getInflatedOccupancy(pos);
}

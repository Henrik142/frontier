#include "frontiers.hpp"

FrontierFinder::FrontierFinder() {    
}

FrontierFinder::~FrontierFinder() {
}

void FrontierFinder::initFrontierFinder(const std::shared_ptr<Map>& map,
        const PerceptionUtils::InitFOVParams& fov_params, 
        const InitFrontierParams& frontier_params) {
    // Initialize map pointer
    map_ = map;

    // Get map params
    resolution_ = getResolution();
    Eigen::Vector2d origin;
    getOrigin(origin);

    // initialize frontier_flag_ vector to the size of the map, all set to 0 (not a frontier)
    int cell_num = map_->getVoxelNum();
    frontier_flag_ = std::vector<char>(cell_num, 0);

    // initialize raycaster
    raycaster_.reset(new RayCaster);
    raycaster_->setParams(resolution_, origin);

    // Initialize perception utils
    percep_utils_.reset(new PerceptionUtils());
    percep_utils_->initPercepUtils(fov_params);

    // Initialize frontier parameters
    update_inflation_size_ = frontier_params.update_inflation_size;
    cluster_max_radius_ = frontier_params.cluster_max_radius;
    candidate_rnum_ = frontier_params.candidate_rnum;
    candidate_rmin_ = frontier_params.candidate_rmin;
    candidate_rmax_ = frontier_params.candidate_rmax;
    candidate_dphi_ = frontier_params.candidate_dphi;
    min_clearance_ = frontier_params.min_clearance;
    min_visib_num_ = frontier_params.min_visib_num;
}

void FrontierFinder::findFrontiers() {
    // Clear the temporary list of frontier clusters before finding new frontiers
    tmp_clusters_.clear();

    // Bounding box of updated region
    Eigen::Vector2i update_min, update_max;
    getUpdatedBox(update_min, update_max, true);

    // Define lambda function for removing outdated clusters
    // resets the frontier_flag_ entry of all cells to 0 and removes the cluster from the list of all clusters
    auto resetFlag = [&](std::list<FrontierCluster>::iterator& iter, std::list<FrontierCluster>& frontiers) {
        for (auto cell : iter->cells_) {
            frontier_flag_[toAddress(cell)] = 0;
        }
        iter = frontiers.erase(iter);
    };

    // Remove outdated clusters
    // cluster is outdated if it overlaps with the updated region and any of its cells is no longer a frontier cell
    removed_ids_.clear();
    for (auto iter = clusters_.begin(); iter != clusters_.end();) {
        if (haveOverlap(iter->box_min_, iter->box_max_, update_min - Eigen::Vector2i(1,1),
                        update_max + Eigen::Vector2i(1,1)) && isClusterOutdated(*iter)) {
            removed_ids_.push_back(iter->id_);
            resetFlag(iter, clusters_);
        } else {
            ++iter;
        }
    }

    // Search new frontiers in box inflated from updated box
    Eigen::Vector2i search_min = update_min;
    Eigen::Vector2i search_max = update_max;
    inflateBox(search_min, search_max, Eigen::Vector2d(update_inflation_size_, update_inflation_size_));
    
    // restrict search box to map bounds
    fitToMap(search_min, search_max);

    // Iterate over the search box and find frontier seed cells
    for (int i = search_min(0); i <= search_max(0); ++i) {
        for (int j = search_min(1); j <= search_max(1); ++j) {
            Eigen::Vector2i cell(i, j);
            if (frontier_flag_[toAddress(cell)] == 0 && isKnownAndFree(cell) && isNeighborUnknown(cell)) {
                // Expand from the seed cell to find a complete frontier cluster
                expandFrontier(cell);
            }
        }
    }
    splitLargeClusters(tmp_clusters_);
}

void FrontierFinder::expandFrontier(const Eigen::Vector2i& seed_cell) {
    // Data for clustering
    std::queue<Eigen::Vector2i> cell_queue;
    std::vector<Eigen::Vector2i> expanded;

    expanded.push_back(seed_cell);
    cell_queue.push(seed_cell);
    frontier_flag_[toAddress(seed_cell)] = 1;

    // Search frontier cluster based on region growing
    while (!cell_queue.empty()) {
        auto cur = cell_queue.front();
        cell_queue.pop();
        auto nbrs = eightNeighbors(cur);
        for (auto nbr : nbrs) {
            // Cell has to qualify as a frontier cell and not already be in the cluster
            int adr = toAddress(nbr);
            if (frontier_flag_[adr] == 1 || !(isKnownAndFree(nbr) && isNeighborUnknown(nbr))) {
                continue;
            }
            expanded.push_back(nbr);
            cell_queue.push(nbr);
            frontier_flag_[adr] = 1;
        }
    }

    // TODO: Maybe add check if cluster is bigger than some minimum size/number of cells
    // Add the cluster to the temporary list of new clusters
    FrontierCluster cluster;
    cluster.cells_ = expanded;
    computeClusterInfo(cluster);
    tmp_clusters_.push_back(cluster);
}

void FrontierFinder::splitLargeClusters(std::list<FrontierCluster>& clusters) {
    std::list<FrontierCluster> splits, tmps;
    for (auto iter = clusters.begin(); iter != clusters.end(); ++iter) {
        // Check if each cluster needs to be split
        if (splitCluster(*iter, splits)) {
            tmps.insert(tmps.end(), splits.begin(), splits.end());
            splits.clear();
        } else
        tmps.push_back(*iter);
    }
    clusters = tmps;
}

bool FrontierFinder::splitCluster(const FrontierCluster& cluster, std::list<FrontierCluster>& splits) {
    // Split a cluster into smaller pieces if it is too large
    bool need_split = false;
    for (const auto& cell : cluster.cells_) {
        Eigen::Vector2d cell_pos;
        indexToPos(cell, cell_pos);
        if ((cell_pos - cluster.average_pos_).norm() > cluster_max_radius_) {
            need_split = true;
            break;
        }
    }
    if (!need_split) return false;

    // Compute principal components
    // Covariance matrix of cells
    Eigen::Matrix2d cov;
    cov.setZero();
    for (auto cell : cluster.cells_) {
        Eigen::Vector2d cell_pos;
        indexToPos(cell, cell_pos);
        Eigen::Vector2d diff = cell_pos - cluster.average_pos_;
        cov += diff * diff.transpose();
    }
    cov /= double(cluster.cells_.size());

    // Find eigenvector that corresponds to maximal eigenvalue
    Eigen::EigenSolver<Eigen::Matrix2d> es(cov);
    auto values = es.eigenvalues().real();
    auto vectors = es.eigenvectors().real();
    int max_idx;
    double max_eigenvalue = -1000000;
    for (int i = 0; i < values.rows(); ++i) {
        if (values[i] > max_eigenvalue) {
        max_idx = i;
        max_eigenvalue = values[i];
        }
    }
    Eigen::Vector2d first_pc = vectors.col(max_idx);

    // Split the cluster into two groups along the first PC
    FrontierCluster ftr1, ftr2;
    for (auto cell : cluster.cells_) {
        Eigen::Vector2d cell_pos;
        indexToPos(cell, cell_pos);
        if ((cell_pos - cluster.average_pos_).dot(first_pc) >= 0)
            ftr1.cells_.push_back(cell);
        else
            ftr2.cells_.push_back(cell);
    }
    computeClusterInfo(ftr1);
    computeClusterInfo(ftr2);

    // Recursive call to split frontier that is still too large
    std::list<FrontierCluster> splits2;
    if (splitCluster(ftr1, splits2)) {
        splits.insert(splits.end(), splits2.begin(), splits2.end());
        splits2.clear();
    } else
        splits.push_back(ftr1);

    if (splitCluster(ftr2, splits2))
        splits.insert(splits.end(), splits2.begin(), splits2.end());
    else
        splits.push_back(ftr2);

    return true;
}

void FrontierFinder::computeClustersToVisit() {
    first_new_cluster_ = clusters_.end();
    int new_num = 0;
    int new_dormant_num = 0;

    // Try to find viewpoints for each cluster and categorize them according to viewpoint number
    for (auto& tmp_cl : tmp_clusters_) {
        // Search viewpoints around cluster
        sampleViewpoints(tmp_cl);
        if (!tmp_cl.viewpoints_.empty()) {
            ++new_num;
            std::list<FrontierCluster>::iterator inserted = clusters_.insert(clusters_.end(), tmp_cl);
            // Sort the viewpoints by coverage of the frontier cells, best view in front
            sort(inserted->viewpoints_.begin(), inserted->viewpoints_.end(),
                [](const Viewpoint& v1, const Viewpoint& v2) { return v1.visib_num_ > v2.visib_num_; });
            // Keep track of the first newly added cluster for later use
            if (first_new_cluster_ == clusters_.end()) first_new_cluster_ = inserted;
        } else {
            // If no valid viewpoint is found, move cluster to dormant list
            // TODO: implement what to do with dormant clusters
            dormant_clusters_.push_back(tmp_cl);
            ++new_dormant_num;
        }
    }

    // Reset indices of frontiers
    int idx = 0;
    for (auto& cl : clusters_) {
        cl.id_ = idx++;
    }
}

// Sample viewpoints around cluster's average position, check coverage to the frontier cells
void FrontierFinder::sampleViewpoints(FrontierCluster& cluster) {
    // Evaluate sample viewpoints on circles, find ones that cover most cells
    for (double rc = candidate_rmin_, dr = (candidate_rmax_ - candidate_rmin_) / candidate_rnum_;
       rc <= candidate_rmax_ + 1e-3; rc += dr) {
        for (double phi = -M_PI; phi < M_PI; phi += candidate_dphi_) {
            const Eigen::Vector2d sample_pos = cluster.average_pos_ + rc * Eigen::Vector2d(cos(phi), sin(phi));
            
            // Discard candidates in occupied or unknown cells
            if (!(getOccupancy(sample_pos) == Map::FREE)) continue;

            // Compute average yaw by considering the direction from the sample position to each frontier cell
            auto& cells = cluster.cells_;
            double avg_yaw = 0.0;
            Eigen::Vector2d mean_dir = Eigen::Vector2d::Zero();
            for (const auto& cell : cells) {
                Eigen::Vector2d cell_pos;
                indexToPos(cell, cell_pos);

                Eigen::Vector2d direction = cell_pos - sample_pos;
                double len = direction.norm();
                if (len > 1e-9) {
                    // could think about skipping the normalization here, then closer cells would be weighted higher
                    mean_dir += direction / len;
                }
            }
            if (mean_dir.norm() > 1e-9) {
                avg_yaw = std::atan2(mean_dir.y(), mean_dir.x());
            }

            // Compute associated robot position
            percep_utils_->setPose(sample_pos, avg_yaw);
            Eigen::Vector2d robot_pos;
            percep_utils_->getRobotPos(robot_pos);

            // Check if robot position is:
            // - in bounding box
            // - at a safe distance from occupied cells
            // - at a safe distance from unknown cells
            if (!(getInflatedOccupancy(robot_pos) == Map::FREE)) continue;

            // Compute the fraction of covered and visible cells
            int visib_num = countVisibleCells(sample_pos, avg_yaw, cells);
            if (visib_num >= min_visib_num_) {
                Viewpoint vp = { sample_pos, avg_yaw, visib_num, robot_pos };
                cluster.viewpoints_.push_back(vp);
            }
        }
    }
}

int FrontierFinder::countVisibleCells(const Eigen::Vector2d& pos, const double& yaw,
                                        const std::vector<Eigen::Vector2i>& cells) {
    int visib_num = 0;
    Eigen::Vector2i idx;
    percep_utils_->setPose(pos, yaw);

    for (const auto& cell : cells) {
        Eigen::Vector2d cell_pos;
        indexToPos(cell, cell_pos);

        // Check if frontier cell is inside FOV
        if (!percep_utils_->isInsideFOV(cell_pos)) continue;

        // Check if the cell is actually visible (not occluded) by using raycasting
        raycaster_->input(cell_pos, pos);
        bool visib = true;
        // Go through all cells on the ray from the sample position to the frontier cell
        while (raycaster_->nextId(idx)) {
            if (!isKnownAndFree(idx)) {
                // interrupt the loop if there is an occupied or unkown cell
                visib = false;
                break;
            }
        }
        if (visib) visib_num += 1;
    }
    return visib_num;
}

void FrontierFinder::getPathForTour(const Eigen::Vector2d& pos,
                                    const std::vector<int>& frontier_ids,
                                    std::vector<Eigen::Vector2d>& path) {
    // Make a cluster_indexer to access the frontier list easier
    std::vector<std::list<FrontierCluster>::iterator> cluster_indexer;
    for (auto it = clusters_.begin(); it != clusters_.end(); ++it)
        cluster_indexer.push_back(it);

    // Compute the path from current pos to the first frontier
    std::vector<Eigen::Vector2d> segment;
    ViewNode::searchPath(pos, cluster_indexer[frontier_ids[0]]->viewpoints_.front().robot_pos_, segment);
    path.insert(path.end(), segment.begin(), segment.end());

    // Get paths of tour passing all clusters
    for (int i = 0; i < frontier_ids.size() - 1; ++i) {
        // Move to path to next cluster
        auto path_iter = cluster_indexer[frontier_ids[i]]->paths_.begin();
        int next_idx = frontier_ids[i + 1];
        for (int j = 0; j < next_idx; ++j) {
            ++path_iter;
        }
        path.insert(path.end(), path_iter->begin(), path_iter->end());
    }
}

void FrontierFinder::getFullCostMatrix(const Eigen::Vector2d& cur_pos,
                                       const Eigen::Vector2d& cur_vel,
                                       const double& cur_yaw,
                                       Eigen::MatrixXd& mat) {
    // Use Asymmetric TSP formulation
    int dimen = clusters_.size();
    mat.resize(dimen + 1, dimen + 1);
    
    // Fill block for clusters
    int i = 1, j = 1;
    for (auto ftr : clusters_) {
      for (auto cs : ftr.costs_) {
        mat(i, j++) = cs;
      }
      ++i;
      j = 1;
    }

    // Fill block from current state to clusters
    mat.leftCols<1>().setZero();
    for (auto ftr : clusters_) {
      Viewpoint vj = ftr.viewpoints_.front();
      std::vector<Eigen::Vector2d> path;
      mat(0, j++) =
          ViewNode::computeCost(cur_pos, vj.robot_pos_, cur_yaw, vj.yaw_, cur_vel, path);
    }
}

void FrontierFinder::updateFrontierCostMatrix() {
    if (!removed_ids_.empty()) {
        // Delete paths to removed clusters, and corresponding costs
        for (auto it = clusters_.begin(); it != first_new_cluster_; ++it) {
            auto cost_iter = it->costs_.begin();
            auto path_iter = it->paths_.begin();
            int iter_idx = 0;
            for (int i = 0; i < removed_ids_.size(); ++i) {
                // Step iterator to the item to be removed
                while (iter_idx < removed_ids_[i]) {
                    ++cost_iter;
                    ++path_iter;
                    ++iter_idx;
                }
                cost_iter = it->costs_.erase(cost_iter);
                path_iter = it->paths_.erase(path_iter);
                // erase() consumed the element at iter_idx, so bump it to stay aligned
                // with the original (pre-erase) indices used by removed_ids_
                ++iter_idx;
            }
        }
        removed_ids_.clear();
    }

    auto updateCost = [](const std::list<FrontierCluster>::iterator& it1, 
        const std::list<FrontierCluster>::iterator& it2) {
        // Search path from old cluster's top viewpoint to new cluster's top viewpoint
        Viewpoint& vui = it1->viewpoints_.front();
        Viewpoint& vuj = it2->viewpoints_.front();
        std::vector<Eigen::Vector2d> path_ij;
        double cost_ij = ViewNode::computeCost(
            vui.robot_pos_, vuj.robot_pos_, vui.yaw_, vuj.yaw_, Eigen::Vector2d(0, 0), path_ij);
        // Insert item for both old and new clusters
        it1->costs_.push_back(cost_ij);
        it1->paths_.push_back(path_ij);
        std::reverse(path_ij.begin(), path_ij.end());
        it2->costs_.push_back(cost_ij);
        it2->paths_.push_back(path_ij);
    };

    // Compute path and cost between old and new clusters
    for (auto it1 = clusters_.begin(); it1 != first_new_cluster_; ++it1) {
        for (auto it2 = first_new_cluster_; it2 != clusters_.end(); ++it2) {
            updateCost(it1, it2);
        }
    }

    // Compute path and cost between new clusters
    for (auto it1 = first_new_cluster_; it1 != clusters_.end(); ++it1) {
        for (auto it2 = it1; it2 != clusters_.end(); ++it2) {
            if (it1 == it2) {
                it1->costs_.push_back(0);
                it1->paths_.push_back({});
            } else {
                updateCost(it1, it2);
            }
        }
    }
}
    
std::list<FrontierCluster> FrontierFinder::getFrontierClusters() {
    return clusters_;
}

// returns only the four direct neighbors (up, down, left, right) of a cell
std::vector<Eigen::Vector2i> FrontierFinder::fourNeighbors(const Eigen::Vector2i& cell) {
    std::vector<Eigen::Vector2i> neighbors;

    std::vector<Eigen::Vector2i> offsets = {
        Eigen::Vector2i(-1, 0), Eigen::Vector2i(1, 0),
        Eigen::Vector2i(0, -1), Eigen::Vector2i(0, 1)
    };

    for (const auto& offset : offsets) {
        addNeighbor(neighbors, cell, offset);
    }

    return neighbors;
}

// return the direct and also the diagonally adjacent neighbors of a cell
std::vector<Eigen::Vector2i> FrontierFinder::eightNeighbors(const Eigen::Vector2i& cell) {
    std::vector<Eigen::Vector2i> neighbors;

    neighbors = fourNeighbors(cell); // add the four direct neighbors first

    // add the four diagonal neighbors
    std::vector<Eigen::Vector2i> offsets = {
        Eigen::Vector2i(-1, -1), Eigen::Vector2i(-1, 1),
        Eigen::Vector2i(1, -1), Eigen::Vector2i(1, 1)
    };

    for (const auto& offset : offsets) {
        addNeighbor(neighbors, cell, offset);
    }

    return neighbors;
}

void FrontierFinder::addNeighbor(std::vector<Eigen::Vector2i>& neighbors, 
        const Eigen::Vector2i& cell, const Eigen::Vector2i& offset) {
    Eigen::Vector2i neighbor = cell + offset;
    if (isInMap(neighbor)) {
        neighbors.push_back(neighbor);
    }
}

bool FrontierFinder::isNeighborUnknown(const Eigen::Vector2i& cell) {
    std::vector<Eigen::Vector2i> neighbors = fourNeighbors(cell);
    for (const auto& neighbor : neighbors) {
        if (getOccupancy(neighbor) == Map::UNKNOWN) return true;
    }
    return false;
}

bool FrontierFinder::isKnownAndFree(const Eigen::Vector2i& cell) {
    return getOccupancy(cell) == Map::FREE;
}

void FrontierFinder::computeClusterInfo(FrontierCluster& cluster) {
    // Compute average position and bounding box of cluster
    cluster.average_pos_.setZero();
    cluster.box_max_ = cluster.cells_.front();
    cluster.box_min_ = cluster.cells_.front();
    for (auto cell : cluster.cells_) {
        Eigen::Vector2d cell_pos;
        indexToPos(cell, cell_pos);
        cluster.average_pos_ += cell_pos;
        for (int i = 0; i < 2; ++i) {
            cluster.box_min_[i] = std::min(cluster.box_min_[i], cell[i]);
            cluster.box_max_[i] = std::max(cluster.box_max_[i], cell[i]);
        }
    }
    cluster.average_pos_ /= double(cluster.cells_.size());
}

bool FrontierFinder::haveOverlap(const Eigen::Vector2i& bmin1, const Eigen::Vector2i& bmax1,
                                 const Eigen::Vector2i& bmin2, const Eigen::Vector2i& bmax2) {
    // Check if two boxes overlap each other
    for (int i = 0; i < 2; ++i) {
        if (bmax1[i] < bmin2[i] || bmin1[i] > bmax2[i]) return false;
    }
    return true;
}

bool FrontierFinder::isClusterOutdated(const FrontierCluster& cluster) {
    // Checks if any cell in the cluster is no longer a frontier cell
    for (const auto& cell : cluster.cells_) {
        if (!isKnownAndFree(cell) || !isNeighborUnknown(cell)) {
            return true;
        }
    }
    return false;
}

/*
bool FrontierFinder::isPositionSafe(const Eigen::Vector2d& pos) {
    // Check if the position is safe for the robot (not too close to occupied or unknown cells)
    Eigen::Vector2i idx;
    posToIndex(pos, idx);
    if (!isInMap(idx)) return false;

    // all cells witin a square box with side length:
    // 2 * clearance_voxels + 1, centered at the position are checked for safety
    // first, the distance is checked so cells in the corners of the square can be skipped
    const int clearance_voxels = ceil(min_clearance_ / getResolution());
    for (int dx = -clearance_voxels; dx <= clearance_voxels; ++dx) {
        for (int dy = -clearance_voxels; dy <= clearance_voxels; ++dy) {
            Eigen::Vector2i neighbor_idx = idx + Eigen::Vector2i(dx, dy);
            Eigen::Vector2d neighbor_pos;
            indexToPos(neighbor_idx, neighbor_pos);
            if ((neighbor_pos - pos).norm() > min_clearance_) continue;
            if (!isInMap(neighbor_idx)) return false;
            if (getOccupancy(neighbor_idx) != Map::FREE) return false;
        }
    }
    return true;    
}
    */

void FrontierFinder::wrapYaw(double& yaw) {
    while (yaw > M_PI) yaw -= 2.0 * M_PI;
    while (yaw < -M_PI) yaw += 2.0 * M_PI;
}


/* Wrappers of Map class */
double FrontierFinder::getResolution() {
    return map_->getResolution();
}

void FrontierFinder::getOrigin(Eigen::Vector2d& origin) {
    origin[0] = map_->getOriginX();
    origin[1] = map_->getOriginY();
}

int FrontierFinder::toAddress(const Eigen::Vector2i& id) {
    return map_->toAddress(id);
}

void FrontierFinder::indexToPos(const Eigen::Vector2i& idx, Eigen::Vector2d& pos) {
    map_->indexToPos(idx, pos);
}

void FrontierFinder::posToIndex(const Eigen::Vector2d& pos, Eigen::Vector2i& idx) {
    map_->posToIndex(pos, idx);
}

bool FrontierFinder::isInMap(const Eigen::Vector2i& idx) {
    return map_->isInMap(idx);
}

int FrontierFinder::getOccupancy(const Eigen::Vector2i& id) {
    return map_->getOccupancy(id);
}

int FrontierFinder::getOccupancy(const Eigen::Vector2d& pos) {
    return map_->getOccupancy(pos);
}

int FrontierFinder::getInflatedOccupancy(const Eigen::Vector2d& pos) {
    return map_->getInflatedOccupancy(pos);
}

void FrontierFinder::getUpdatedBox(Eigen::Vector2i& bmin, Eigen::Vector2i& bmax, bool reset) {
    map_->getUpdatedBox(bmin, bmax, reset);
}

void FrontierFinder::inflateBox(Eigen::Vector2i& bmin, Eigen::Vector2i& bmax, const Eigen::Vector2d& inflation_size) {
    map_->inflateBox(bmin, bmax, inflation_size);
}

void FrontierFinder::fitToMap(Eigen::Vector2i& bmin, Eigen::Vector2i& bmax) {
    map_->fitToMap(bmin, bmax);
}
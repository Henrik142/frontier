#pragma once

#include <Eigen/Eigen>
#include <queue>
#include "map.hpp"
#include "perception_utils.hpp"
#include "raycast.hpp"
#include "viewpoint_graph.hpp"

// Viewpoint to cover a frontier cluster
struct Viewpoint {
    // Sensor position
    Eigen::Vector2d pos_;
    // Heading of the sensor
    double yaw_;
    // Number of visible frontier cells
    int visib_num_;
    // Associated robot position
    Eigen::Vector2d robot_pos_;
};

struct FrontierCluster {
    // Voxels belonging to the cluster
    std::vector<Eigen::Vector2i> cells_;

    // bounding box of the cluster in voxel indices
    Eigen::Vector2i box_min_, box_max_;

    // the average position/centroid of the cluster in the map
    Eigen::Vector2d average_pos_;

    // Viewpoints that can cover the cluster
    std::vector<Viewpoint> viewpoints_;

    // Index of the cluster, used for identification (starting at 0)
    int id_;

    // Path and cost from this cluster to other clusters
    std::list<std::vector<Eigen::Vector2d>> paths_;
    std::list<double> costs_;
};

class FrontierFinder {
public:
    FrontierFinder();
    ~FrontierFinder();

    struct InitFrontierParams {
        double update_inflation_size;
        double cluster_max_radius;
        int candidate_rnum;
        double candidate_rmin;
        double candidate_rmax;
        double candidate_dphi;
        double min_clearance;
        int min_visib_num;
    };

    void initFrontierFinder(const std::shared_ptr<Map>& map, 
        const PerceptionUtils::InitFOVParams& fov_params,
        const InitFrontierParams& frontier_params);

    void findFrontiers();
    void computeClustersToVisit();

    void getPathForTour(const Eigen::Vector2d& pos,
                        const std::vector<int>& frontier_ids,
                        std::vector<Eigen::Vector2d>& path);
    void getFullCostMatrix(const Eigen::Vector2d& cur_pos,
                           const Eigen::Vector2d& cur_vel,
                           const double& cur_yaw,
                           Eigen::MatrixXd& mat);
    void updateFrontierCostMatrix();

    std::list<FrontierCluster> getFrontierClusters();   

private:
    void expandFrontier(const Eigen::Vector2i& seed_cell);
    void splitLargeClusters(std::list<FrontierCluster>& clusters);
    bool splitCluster(const FrontierCluster& cluster, std::list<FrontierCluster>& splits);
    void sampleViewpoints(FrontierCluster& cluster);
    int countVisibleCells(const Eigen::Vector2d& pos, const double& yaw, const std::vector<Eigen::Vector2i>& cells);

    std::vector<Eigen::Vector2i> fourNeighbors(const Eigen::Vector2i& cell);
    std::vector<Eigen::Vector2i> eightNeighbors(const Eigen::Vector2i& cell);
    void addNeighbor(std::vector<Eigen::Vector2i>& neighbors, const Eigen::Vector2i& cell, const Eigen::Vector2i& offset);
    bool isNeighborUnknown(const Eigen::Vector2i& cell);
    bool isKnownAndFree(const Eigen::Vector2i& cell);
    void computeClusterInfo(FrontierCluster& ftr);
    bool haveOverlap(const Eigen::Vector2i& bmin1, const Eigen::Vector2i& bmax1,
                     const Eigen::Vector2i& bmin2, const Eigen::Vector2i& bmax2);
    bool isClusterOutdated(const FrontierCluster& cluster);
    bool isPositionSafe(const Eigen::Vector2d& pos);
    void wrapYaw(double& yaw);

    // Wrapper of Map class
    double getResolution();
    void getOrigin(Eigen::Vector2d& origin);
    int toAddress(const Eigen::Vector2i& id);
    void indexToPos(const Eigen::Vector2i& idx, Eigen::Vector2d& pos);
    void posToIndex(const Eigen::Vector2d& pos, Eigen::Vector2i& idx);
    bool isInMap(const Eigen::Vector2i& idx);
    int getOccupancy(const Eigen::Vector2i& id);
    int getOccupancy(const Eigen::Vector2d& pos);
    int getInflatedOccupancy(const Eigen::Vector2d& pos);
    void getUpdatedBox(Eigen::Vector2i& bmin, Eigen::Vector2i& bmax, bool reset = false);
    void inflateBox(Eigen::Vector2i& bmin, Eigen::Vector2i& bmax, const Eigen::Vector2d& inflation_size);
    void fitToMap(Eigen::Vector2i& bmin, Eigen::Vector2i& bmax);
    
    // Data
    // List of currently active frontier clusters.
    // They have valid viewpoints associated with them and are thus considered for exploration.
    std::list<FrontierCluster> clusters_;

    // List of frontier clusters that are currently dormant.
    // They have no valid viewpoints associated with them and are thus not considered for exploration.
    std::list<FrontierCluster> dormant_clusters_;

    // Temporary list of frontier clusters used during processing.
    // They are either added to the list of active or dormant clusters.
    std::list<FrontierCluster> tmp_clusters_;
    
    // Flag indicating whether each cell is part of a frontier cluster.
    // 0: not a frontier, 1: frontier
    std::vector<char> frontier_flag_;

    std::list<FrontierCluster>::iterator first_new_cluster_;
    
    // List of IDs of removed frontier clusters.
    std::vector<int> removed_ids_;

    // Params
    // Map resolution
    double resolution_;

    // Sensor update box is inflated by this value in x and y directions when searching for new frontiers
    double update_inflation_size_;

    // Maximum allowed radius of a frontier cluster.
    // If any cell is further away from the centroid than this radius, the cluster is split up.
    double cluster_max_radius_;
    
    // Angular increment between sampled viewpoints around the cluster centroid
    double candidate_dphi_;
    // Number of sampled viewpoints at each angle (sampled radially)
    int candidate_rnum_;
    // Total number of sampled viewpoints corresponds to:
    // candidate_rnum_ * (2 * M_PI / candidate_dphi_)

    // Minimum and maximum radius/distance of sampled viewpoints from cluster centroid
    double candidate_rmax_, candidate_rmin_;
    
    // Minimum clearance that a viewpoint must have from occupied or unknown cells
    double min_clearance_;
    
    // Minimum number of visible frontier cells required for a viewpoint to be considered valid
    int min_visib_num_;
    
    // Utils
    std::shared_ptr<Map> map_;
    std::shared_ptr<PerceptionUtils> percep_utils_;
    std::unique_ptr<RayCaster> raycaster_;

};
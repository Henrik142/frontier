#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sstream>
#include <string>
#include <memory>
#include <atomic>
#include <cstdlib>
#include <fstream>
#include <filesystem>
#include <unistd.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <Eigen/Eigen>

#include "map.hpp"
#include "planning_maps.hpp"
#include "raycast.hpp"
#include "frontiers.hpp"
#include "viewpoint_graph.hpp"
#include "visualization_utils.hpp"
#include "perception_utils.hpp"
#include "astar.hpp"
#include "dijkstra_search.hpp"
#include "lkh_tsp_solver/lkh_interface.h"

enum EXPL_RESULT { WAITING_FOR_INIT, NO_FRONTIER, NO_TOUR, FAIL, SUCCEED };

// Plans a global tour across the frontier clusters found by FrontierUpdater.
class PathPlanningManager : public rclcpp::Node
{
public:
    // Takes the map owned by MapUpdater so both nodes operate on the same instance.
    PathPlanningManager(const std::shared_ptr<PlanningMaps> & planning_maps,
                        const std::shared_ptr<std::atomic_bool> & map_ready);
    ~PathPlanningManager();

private:
    void initParams();
    void pathPlanningInit();
    int pathPlanningCallback();
    void findGlobalTour(std::vector<int> & tour, const Eigen::MatrixXd & cost_mat);
    void refineLocalTour(const Eigen::Vector2d& cur_pos,
                         const Eigen::Vector2d& cur_vel,
                         const double& cur_yaw,
                         const std::vector<std::vector<Eigen::Vector2d>>& n_points,
                         const std::vector<std::vector<double>>& n_yaws,
                         std::vector<Eigen::Vector2d>& refined_pts,
                         std::vector<double>& refined_yaws);
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr odometry_msg);
    void publishFrontierStatus(const std::list<FrontierCluster> & frontier_clusters,
                               bool empty = false);
    void publishPathplanningStatus(const std::vector<Eigen::Vector2d>& global_path,
                                   const Eigen::Vector2d& next_pos,
                                   const double next_yaw,
                                   const std::vector<Eigen::Vector2d>& refined_path,
                                   bool empty = false);
    void publishEmptyMarkers();
    double wrapYaw(double yaw);

    // Separate callback groups for path planning and odometry callbacks
    rclcpp::CallbackGroup::SharedPtr cb_group_path_planning_;
    rclcpp::CallbackGroup::SharedPtr cb_group_odometry_;

    // Timers for path planning
    rclcpp::TimerBase::SharedPtr path_planning_init_timer_;
    rclcpp::TimerBase::SharedPtr path_planning_timer_;

    // Subscribers and publishers

    // Odometry subscription
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odometry_;

    // Target trajectory and yaw publishers for trajectory generation
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr publisher_pos_target_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_yaw_target_;

    // Publishers for log output and rviz visualization
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_global_path_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_refined_views_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_next_viewpoint_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_frontiers_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_viewpoints_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_cluster_centroids_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_fov_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_velocity_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_string_;

    // Params
    std::string vehicle_name_;
    PerceptionUtils::InitFOVParams fov_params_;
    Astar::InitAstarParams path_astar_params_;
    FrontierFinder::InitFrontierParams frontier_params_;

    // Maximum search times for global and local A* searches (in seconds, for each path)
    double max_search_time_global_;
    double max_search_time_local_;

    // Flag to decide if the global tour should be computed greedily (true) or by
    // solving the ATSP (false)
    double global_tour_greedy_;

    // Flag to enable local tour refinement
    bool refine_local_;
    // Max. number and max. distance from robot of clusters to consider in refinement
    int refine_num_clusters_;
    double refine_radius_;
    // Number of viewpoints to consider for each cluster and the
    // maximum decay factor for refinement
    int refine_num_viewpoints_, refine_num_viewpoints_first_;
    double refine_max_decay_;

    // Maximum velocity, maximum yaw rate, weight for yaw change,
    // weight for motion consistency cost
    double vm_, yd_, w_y_, w_dir_;

    // Update rate for the path planning loop (in milliseconds)
    int update_rate_ms_;

    // Parameter file for LKH
    std::string data_dir_;

    // Utils
    std::shared_ptr<VisualizationUtils> vis_utils_;
    std::vector<std::array<double, 3>> frontier_colors_;
    std::shared_ptr<FrontierFinder> frontier_finder_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Data
    std::shared_ptr<PlanningMaps> planning_maps_;
    std::shared_ptr<const Map> most_recent_map_;
    std::shared_ptr<std::atomic_bool> map_ready_;
    
    // Robot state
    Eigen::Vector2d hippo_position_;
    Eigen::Vector2d hippo_velocity_;
    double hippo_yaw_;

    // Initial yaw of the robot at the start of the initialization rotation
    double hippo_yaw_start_;

    // Time values
    double frontier_max_time_ = 0.0;
    double dijkstra_max_time_ = 0.0;
    double tsp_max_time_ = 0.0;
    double pathplanning_max_time_ = 0.0;

    // flags for odometry and initialization rotation
    bool odometry_received_ = false;
    bool init_rotation_started_ = false;
    bool init_rotation_halfway_ = false;
    bool init_rotation_finished_ = false;
    bool path_planning_initialized_ = false;
};

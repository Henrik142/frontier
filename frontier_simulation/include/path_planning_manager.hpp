#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sstream>
#include <string>
#include <memory>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <Eigen/Eigen>

#include "map.hpp"
#include "map_builder.hpp"
#include "raycast.hpp"
#include "frontiers.hpp"
#include "viewpoint_graph.hpp"
#include "visualization_utils.hpp"
#include "perception_utils.hpp"
#include "astar.hpp"
#include "dijkstra_search.hpp"
#include "trajectory_planner.hpp"

enum EXPL_RESULT { WAITING_FOR_INIT, NO_FRONTIER, NO_TOUR, FAIL, SUCCEED };

// Plans a global tour across the frontier clusters found by FrontierUpdater.
class PathPlanningManager : public rclcpp::Node
{
public:
    // Takes the map owned by MapUpdater so both nodes operate on the same instance.
    PathPlanningManager(const std::shared_ptr<Map> & map);
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
    void shortenPath(std::vector<Eigen::Vector2d>& path);
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr odometry_msg);
    void publishFrontierStatus(const std::list<FrontierCluster> & frontier_clusters);
    double wrapYaw(double yaw);

    // Subscribers and publishers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odometry_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_trajectory_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_global_path_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_path_to_next_goal_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_refined_views_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_next_viewpoint_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_frontiers_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_viewpoints_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_top_robot_positions_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_fov_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_velocity_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_string_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_yaw_;

    // Params
    std::string vehicle_name_;
    PerceptionUtils::InitFOVParams fov_params_;
    Astar::InitAstarParams path_astar_params_, trajectory_astar_params_;
    FrontierFinder::InitFrontierParams frontier_params_;

    // Distance threshold for consecutive points for path shortening
    double shorten_path_dist_thresh_;

    // Flag to enable local tour refinement
    bool refine_local_;
    // Max. number and max. distance from robot of clusters to consider in refinement
    int refine_num_;
    double refine_radius_;

    // Maximum velocity, maximum yaw rate, weight for yaw change,
    // weight for motion consistency cost
    double vm_, yd_, w_y_, w_dir_;

    // Update rate for the path planning loop (in milliseconds)
    int update_rate_ms_;

    // Utils
    std::shared_ptr<VisualizationUtils> vis_utils_;
    rclcpp::TimerBase::SharedPtr path_planning_init_timer_;
    rclcpp::TimerBase::SharedPtr path_planning_timer_;
    std::vector<std::array<double, 3>> frontier_colors_;
    std::shared_ptr<FrontierFinder> frontier_finder_;
    std::shared_ptr<TrajectoryPlanner> trajectory_planner_;
    std::shared_ptr<MapBuilder> map_builder_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Data (owned by MapUpdater, shared here)
    std::shared_ptr<Map> map_;
    
    // Robot state
    Eigen::Vector2d hippo_position_;
    Eigen::Vector2d hippo_velocity_;
    double hippo_yaw_;

    // Initial yaw of the robot at the start of the initialization rotation
    double hippo_yaw_start_;

    // flags for odometry and initialization rotation
    bool odometry_received_ = false;
    bool init_box_discovered_ = false;
    bool init_rotation_started_ = false;
    bool init_rotation_halfway_ = false;
    bool init_rotation_finished_ = false;
    bool path_planning_initialized_ = false;
};

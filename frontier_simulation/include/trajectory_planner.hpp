#pragma once

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "map.hpp"
#include "planning_maps.hpp"
#include "raycast.hpp"
#include "perception_utils.hpp"
#include "visualization_utils.hpp"
#include "astar.hpp"

class TrajectoryPlanner : public rclcpp::Node {
public:
    TrajectoryPlanner(const std::shared_ptr<PlanningMaps> & planning_maps);
    ~TrajectoryPlanner();

private:
    void initParams();

    void planTrajectory();
    void shortenPath(std::vector<Eigen::Vector2d>& path);
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr odometry_msg);
    void posTargetCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr pos_target_msg);

    // Subscribers and publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_trajectory_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_path_to_next_goal_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odometry_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr subscription_pos_target_;

    // Params
    std::string vehicle_name_;
    Astar::InitAstarParams astar_params_;
    PerceptionUtils::InitFOVParams fov_params_;

    // Update rate for the trajectory planning loop (in milliseconds)
    int update_rate_ms_;

    // Distance threshold for consecutive points for path shortening
    double shorten_path_dist_thresh_;

    // Data
    std::shared_ptr<PlanningMaps> planning_maps_;
    std::shared_ptr<const Map> most_recent_map_;

    // Robot state
    Eigen::Vector2d hippo_position_;

    // Target position
    Eigen::Vector2d next_pos_;

    // Utils
    rclcpp::TimerBase::SharedPtr trajectory_timer_;
    std::unique_ptr<Astar> astar_;
    std::unique_ptr<RayCaster> raycaster_;
    std::shared_ptr<VisualizationUtils> vis_utils_;

    // flags for odometry and target position
    bool odometry_received_ = false;
    bool pos_target_received_ = false;
};
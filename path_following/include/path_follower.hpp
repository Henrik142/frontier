#pragma once

#include <rclcpp/rclcpp.hpp>
#include <hippo_control_msgs/msg/roll_target.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64.hpp>
#include <Eigen/Eigen>

class PathFollower : public rclcpp::Node {
public:
    PathFollower();
    ~PathFollower();

private:
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr odometry_msg);
    void pathCallback(const nav_msgs::msg::Path::SharedPtr path_msg);
    void yawCallback(const std_msgs::msg::Float64::SharedPtr yaw_msg);

    // Subscribers and publishers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odometry_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscription_path_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_next_yaw_;

    rclcpp::Publisher<hippo_control_msgs::msg::RollTarget>::SharedPtr publisher_roll_target_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_position_target_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr publisher_heading_target_;

    // Params
    double roll_setpoint_;
    double velocity_setpoint_;
    double min_target_dist_;

    // Data
    std::vector<Eigen::Vector2d> current_path_;
    double next_yaw_ = 0.0;
    bool path_received_ = false;

};
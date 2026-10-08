#pragma once

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <hippo_control_msgs/msg/actuator_setpoint.hpp>

class VelocityControl : public rclcpp::Node {
public:
    VelocityControl();
    ~VelocityControl();

private:
    void initParams();
    
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr odometry_msg);
    void velocityTargetCallback(const std_msgs::msg::Float64::SharedPtr velocity_target_msg);

    // Subscriptions and Publishers
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_velocity_target_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odometry_;

    rclcpp::Publisher<hippo_control_msgs::msg::ActuatorSetpoint>::SharedPtr publisher_thrust_setpoint_;

    // Params
    double kp_;

    // Data
    double velocity_target_;
    bool target_received_ = false;
};
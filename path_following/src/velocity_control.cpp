#include "velocity_control.hpp"

VelocityControl::VelocityControl() : Node("velocity_control") {
    initParams();

    publisher_thrust_setpoint_ = this->create_publisher<hippo_control_msgs::msg::ActuatorSetpoint>(
        "thrust_setpoint", rclcpp::SensorDataQoS());

    subscription_velocity_target_ = this->create_subscription<std_msgs::msg::Float64>(
        "velocity_target", 10, std::bind(&VelocityControl::velocityTargetCallback, this, std::placeholders::_1));

    subscription_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry", 10, std::bind(&VelocityControl::odometryCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Node started");
}

VelocityControl::~VelocityControl() {
}

void VelocityControl::initParams() {
    declare_parameter<double>("velocity_control.kp", 1.0);

    kp_ = get_parameter("velocity_control.kp").as_double();
    RCLCPP_INFO(get_logger(), "velocity_control.kp=%f", kp_);
}

void VelocityControl::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr odometry_msg) {
    // Get the current velocity in the x direction of the body frame
    double hippo_velocity_x = odometry_msg->twist.twist.linear.x;

    if (!target_received_) {
        return;
    }

    double vel_error_x = velocity_target_ - hippo_velocity_x;
    
    hippo_control_msgs::msg::ActuatorSetpoint thrust_setpoint_msg;
    thrust_setpoint_msg.header.stamp = this->now();
    thrust_setpoint_msg.header.frame_id = odometry_msg->child_frame_id;
    thrust_setpoint_msg.x = kp_ * vel_error_x;
    thrust_setpoint_msg.ignore_y = true;
    thrust_setpoint_msg.ignore_z = true;
    publisher_thrust_setpoint_->publish(thrust_setpoint_msg);
}

void VelocityControl::velocityTargetCallback(const std_msgs::msg::Float64::SharedPtr velocity_target_msg) {
    velocity_target_ = velocity_target_msg->data;
    target_received_ = true;
}
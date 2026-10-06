#include "path_follower.hpp"

PathFollower::PathFollower() : Node("path_follower") {
    publisher_roll_target_ = this->create_publisher<hippo_control_msgs::msg::RollTarget>("roll_target", 10);
    publisher_position_target_ = this->create_publisher<geometry_msgs::msg::PointStamped>("position_target", 10);
    publisher_heading_target_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("heading_target", 10);

    subscription_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry",
        rclcpp::SensorDataQoS(),   // sensor data QoS: best-effort, suited for high-rate streams
        std::bind(&PathFollower::odometryCallback, this, std::placeholders::_1));

    subscription_path_ = this->create_subscription<nav_msgs::msg::Path>(
        "frontier/trajectory",
        rclcpp::SensorDataQoS(),   // sensor data QoS: best-effort, suited for high-rate streams
        std::bind(&PathFollower::pathCallback, this, std::placeholders::_1));

    subscription_next_yaw_ = this->create_subscription<std_msgs::msg::Float64>(
        "frontier/next_yaw",
        rclcpp::SensorDataQoS(),   // sensor data QoS: best-effort, suited for high-rate streams
        std::bind(&PathFollower::yawCallback, this, std::placeholders::_1));

    roll_setpoint_ = 0.0;
    min_target_dist_ = 0.05;

    // Currently unused
    velocity_setpoint_ = 0.3;

    RCLCPP_INFO(this->get_logger(), "Node started");
}

PathFollower::~PathFollower() {
}

void PathFollower::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr odometry_msg) {
    // Only run if there is a received path
    if (!path_received_) {
        return;
    }

    // roll target is constant
    hippo_control_msgs::msg::RollTarget roll_target_msg;
    roll_target_msg.header.stamp = this->now();
    roll_target_msg.roll_target = roll_setpoint_;
    publisher_roll_target_->publish(roll_target_msg);

    Eigen::Vector3d hippo_position;
    hippo_position << odometry_msg->pose.pose.position.x, 
                      odometry_msg->pose.pose.position.y,
                      odometry_msg->pose.pose.position.z;

    // Set the target depth to be the same as the hippo's current depth
    // to prevent any pitch motion
    double target_depth = hippo_position.z();
    
    // Simple carrot chaser that selects the next waypoint on the path as the target
    // It is expected that the path has at least two points.
    // The first point is exptected to be the robot's own position.
    Eigen::Vector3d target_position;
    target_position << current_path_[1].x(), current_path_[1].y(), target_depth;

    Eigen::Vector3d viewpoint_position;
    viewpoint_position << current_path_.back().x(), current_path_.back().y(), target_depth;

    // Look in the direction of the viewpoint, if sufficiently close to viewpoint
    if ((viewpoint_position - hippo_position).norm() < 0.05) {
        target_position.x() = std::cos(next_yaw_);
        target_position.y() = std::sin(next_yaw_);
    // Jump to next waypoint if the first one is too close
    } else if ((target_position - hippo_position).norm() < min_target_dist_
        && current_path_.size() > 2) {
        target_position.x() = current_path_[2].x();
        target_position.y() = current_path_[2].y();
    }

    // Publish target position for rviz
    geometry_msgs::msg::PointStamped position_target_msg;
    position_target_msg.header.stamp = this->now();
    position_target_msg.header.frame_id = "map";
    position_target_msg.point.x = target_position.x();
    position_target_msg.point.y = target_position.y();
    position_target_msg.point.z = 0.0;
    publisher_position_target_->publish(position_target_msg);

    Eigen::Vector3d target_heading = (target_position - hippo_position).normalized();

    // Publish target heading for the controller
    geometry_msgs::msg::Vector3Stamped target_heading_msg;
    target_heading_msg.header.stamp = position_target_msg.header.stamp;
    target_heading_msg.header.frame_id = "map";
    target_heading_msg.vector.x = target_heading.x();
    target_heading_msg.vector.y = target_heading.y();
    target_heading_msg.vector.z = target_heading.z();
    publisher_heading_target_->publish(target_heading_msg);


    /*
    // Question: Set velocity target in the direction that the hippo is facing,
    // or in the direction of the target point, or in the path direction?

    // velocity target is set in the Hippo's body frame
    geometry_msgs::msg::Vector3Stamped velocity_target_msg;
    velocity_target_msg.header.stamp = position_target_msg.header.stamp;
    velocity_target_msg.vector.x = velocity_setpoint_;
    velocity_target_msg.vector.y = 0.0;
    velocity_target_msg.vector.z = 0.0;
    publisher_velocity_target_->publish(velocity_target_msg);
    */
}

void PathFollower::pathCallback(const nav_msgs::msg::Path::SharedPtr path_msg) {
    current_path_.clear();

    for (const auto& pose_stamped : path_msg->poses) {
        Eigen::Vector2d point;
        const auto& position = pose_stamped.pose.position;
        point << position.x, position.y;
        current_path_.push_back(point);
    }

    // It is expected that the path has at least two points.
    // The first point is exptected to be the robot's own position.
    if (current_path_.size() > 1) {
        path_received_ = true;
    } else {
        path_received_ = false;
    }
}

void PathFollower::yawCallback(const std_msgs::msg::Float64::SharedPtr yaw_msg) {
    // Store the yaw of the next viewpoint
    next_yaw_ = yaw_msg->data;
}
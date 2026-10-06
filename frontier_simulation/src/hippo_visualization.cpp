#include "hippo_visualization.hpp"

HippoVisualization::HippoVisualization() : rclcpp::Node("hippo_visualization") {
    vehicle_name_ = this->declare_parameter<std::string>("vehicle_name", "");
    initParams();

    vis_utils_.reset(new VisualizationUtils());
    vis_utils_->initVisUtils(fov_params_);

    percep_utils_.reset(new PerceptionUtils());
    percep_utils_->initPercepUtils(fov_params_);

    publisher_robot_pose_ = this->create_publisher<visualization_msgs::msg::Marker>("robot_pose_marker", 10);
    publisher_fov_ = this->create_publisher<visualization_msgs::msg::Marker>("robot_fov_marker", 10);
    publisher_target_ = this->create_publisher<visualization_msgs::msg::Marker>("target_marker", 10);

    subscription_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry",
        rclcpp::SensorDataQoS(),
        std::bind(&HippoVisualization::odometryCallback, this, std::placeholders::_1));

    subscription_position_target_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "position_target",
        rclcpp::SensorDataQoS(),
        std::bind(&HippoVisualization::positionTargetCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Node started");
}

HippoVisualization::~HippoVisualization() {
}

void HippoVisualization::initParams() {
    // FOV parameters
    declare_parameter<double>("fov.offset_x", 0.2);
    declare_parameter<double>("fov.offset_y", 0.0);
    declare_parameter<double>("fov.left_angle", -45.0);
    declare_parameter<double>("fov.right_angle", 45.0);
    declare_parameter<double>("fov.max_dist", 0.6);

    fov_params_.offset_x = get_parameter("fov.offset_x").as_double();
    RCLCPP_INFO(get_logger(), "fov.offset_x=%f", fov_params_.offset_x);
    fov_params_.offset_y = get_parameter("fov.offset_y").as_double();
    RCLCPP_INFO(get_logger(), "fov.offset_y=%f", fov_params_.offset_y);
    fov_params_.left_angle = get_parameter("fov.left_angle").as_double();
    RCLCPP_INFO(get_logger(), "fov.left_angle=%f", fov_params_.left_angle);
    fov_params_.right_angle = get_parameter("fov.right_angle").as_double();
    RCLCPP_INFO(get_logger(), "fov.right_angle=%f", fov_params_.right_angle);
    fov_params_.max_dist = get_parameter("fov.max_dist").as_double();
    RCLCPP_INFO(get_logger(), "fov.max_dist=%f", fov_params_.max_dist);

    fov_params_.left_angle = fov_params_.left_angle * M_PI / 180.0;
    fov_params_.right_angle = fov_params_.right_angle * M_PI / 180.0;
}

void HippoVisualization::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & odometry_msg) {
    // Create and publish the marker that represents the robot
    visualization_msgs::msg::Marker marker_pos;
    marker_pos.header.frame_id = "map";
    marker_pos.header.stamp = rclcpp::Clock().now();

    Eigen::Vector2d robot_position, camera_position;
    double yaw;

    robot_position << odometry_msg->pose.pose.position.x, odometry_msg->pose.pose.position.y;
    yaw = tf2::getYaw(odometry_msg->pose.pose.orientation);

    vis_utils_->drawRobotPose(marker_pos, robot_position, yaw);
    publisher_robot_pose_->publish(marker_pos);

    // Create and publish the marker that represents the robot's field of view
    percep_utils_->getCameraPosFromRobotPose(camera_position, robot_position, yaw);
    visualization_msgs::msg::Marker marker_fov;
    marker_fov.header.frame_id = "map";
    marker_fov.header.stamp = rclcpp::Clock().now();

    vis_utils_->drawRobotFOV(marker_fov, camera_position, yaw);
    publisher_fov_->publish(marker_fov);
}

void HippoVisualization::positionTargetCallback(const geometry_msgs::msg::PointStamped::ConstSharedPtr & position_target_msg) {
    // Create and publish the marker that represents the target position
    visualization_msgs::msg::Marker marker_target;
    marker_target.header.frame_id = "map";
    marker_target.header.stamp = this->now();

    Eigen::Vector2d target_position;
    target_position << position_target_msg->point.x, position_target_msg->point.y;
    vis_utils_->drawTarget(marker_target, target_position);
    publisher_target_->publish(marker_target);
}
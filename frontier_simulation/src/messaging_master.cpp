#include "messaging_master.hpp"

MessagingMaster::MessagingMaster() : rclcpp::Node("messaging_master")
{
    initParams();

    hippo_positions_.assign(num_robots_, Eigen::Vector2d::Zero());

    initOdometrySubscriptions();
    initPublishers();

    subscription_chunk_stamps_ = this->create_subscription<environment_map_msgs::msg::ChunkStamps>(
        "/multi_map_manager/chunk_stamps",
        10,
        std::bind(&MessagingMaster::chunkStampsCallback, this, std::placeholders::_1)
    );
    
    subscription_chunk_data_ = this->create_subscription<environment_map_msgs::msg::ChunkData>(
        "/multi_map_manager/chunk_data",
        10,
        std::bind(&MessagingMaster::chunkDataCallback, this, std::placeholders::_1)
    );
}

MessagingMaster::~MessagingMaster() {
}

void MessagingMaster::initOdometrySubscriptions() {
    subscription_odometry_.resize(num_robots_);
    for (int i = 0; i < num_robots_; ++i) {
        std::string topic = std::string("/uuv") + (i < 10 ? "0" : "") + std::to_string(i) + "/odometry";
        subscription_odometry_[i] = this->create_subscription<nav_msgs::msg::Odometry>(
            topic,
            10,
            [this, i](const nav_msgs::msg::Odometry::SharedPtr msg) {
                this->odometryCallback(msg, i);
            }
        );
        RCLCPP_INFO(get_logger(), "Subscribed to: %s", topic.c_str());
    }
}

void MessagingMaster::initPublishers() {
    publishers_chunk_stamps_.resize(num_robots_);
    publishers_chunk_data_.resize(num_robots_);
    for (int i = 0; i < num_robots_; ++i) {
        std::string chunk_stamps_topic = std::string("/uuv") + (i < 10 ? "0" : "") + std::to_string(i) + "/chunk_stamps_filtered";
        publishers_chunk_stamps_[i] = this->create_publisher<environment_map_msgs::msg::ChunkStamps>(chunk_stamps_topic, rclcpp::QoS(1).transient_local());
        RCLCPP_INFO(get_logger(), "Advertising: %s", chunk_stamps_topic.c_str());

        std::string chunk_data_topic = std::string("/uuv") + (i < 10 ? "0" : "") + std::to_string(i) + "/chunk_data_filtered";
        publishers_chunk_data_[i] = this->create_publisher<environment_map_msgs::msg::ChunkData>(chunk_data_topic, rclcpp::QoS(1).transient_local());
        RCLCPP_INFO(get_logger(), "Advertising: %s", chunk_data_topic.c_str());
    }
}

void MessagingMaster::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg, int robot_id) {
    if (robot_id < 0 || robot_id >= num_robots_) {
        RCLCPP_WARN(get_logger(), "Received odometry for invalid robot_id: %d", robot_id);
        return;
    }
    
    hippo_positions_[robot_id].x() = msg->pose.pose.position.x;
    hippo_positions_[robot_id].y() = msg->pose.pose.position.y;
}

void MessagingMaster::chunkStampsCallback(const environment_map_msgs::msg::ChunkStamps::SharedPtr msg) {
    Eigen::Vector2d from_robot_position;
    from_robot_position.x() = hippo_positions_[msg->from_robot_id].x();
    from_robot_position.y() = hippo_positions_[msg->from_robot_id].y();

    for (int i=0; i<num_robots_; ++i) {
        if (i == msg->from_robot_id) continue;
        Eigen::Vector2d to_robot_position = hippo_positions_[i];
        double distance = (from_robot_position - to_robot_position).norm();
        if (distance <= communication_range_) {
            publishers_chunk_stamps_[i]->publish(*msg);
        }
    }
}

void MessagingMaster::chunkDataCallback(const environment_map_msgs::msg::ChunkData::SharedPtr msg) {
    Eigen::Vector2d from_robot_position;
    from_robot_position.x() = hippo_positions_[msg->from_robot_id].x();
    from_robot_position.y() = hippo_positions_[msg->from_robot_id].y();

    for (int i=0; i<num_robots_; ++i) {
        if (i == msg->from_robot_id) continue;
        Eigen::Vector2d to_robot_position = hippo_positions_[i];
        double distance = (from_robot_position - to_robot_position).norm();
        if (distance <= communication_range_) {
            publishers_chunk_data_[i]->publish(*msg);
        }
    }
}

void MessagingMaster::initParams() {
    declare_parameter<int>("num_robots", 2);
    num_robots_ = get_parameter("num_robots").as_int();
    RCLCPP_INFO(get_logger(), "num_robots=%d", num_robots_);

    declare_parameter<double>("communication_range", 2.0);
    communication_range_ = get_parameter("communication_range").as_double();
    RCLCPP_INFO(get_logger(), "communication_range=%f", communication_range_);
}
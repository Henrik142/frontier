#pragma once

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <nav_msgs/msg/odometry.hpp>
#include <random>

#include "environment_map_msgs/msg/chunk_data.hpp"
#include "environment_map_msgs/msg/chunk_stamps.hpp"

class MessagingMaster : public rclcpp::Node
{
public:
    MessagingMaster();
    ~MessagingMaster();

private:
    void initParams();
    void initOdometrySubscriptions();
    void initPublishers();
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg, int robot_id);
    void chunkStampsCallback(const environment_map_msgs::msg::ChunkStamps::SharedPtr msg);
    void chunkDataCallback(const environment_map_msgs::msg::ChunkData::SharedPtr msg);
    double randomDouble();

    // Subscribers and Publishers
    rclcpp::Subscription<environment_map_msgs::msg::ChunkStamps>::SharedPtr subscription_chunk_stamps_;
    rclcpp::Subscription<environment_map_msgs::msg::ChunkData>::SharedPtr subscription_chunk_data_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> subscription_odometry_;

    std::vector<rclcpp::Publisher<environment_map_msgs::msg::ChunkStamps>::SharedPtr> publishers_chunk_stamps_;
    std::vector<rclcpp::Publisher<environment_map_msgs::msg::ChunkData>::SharedPtr> publishers_chunk_data_;

    // Params
    int num_robots_;
    double communication_range_;
    double msg_reliability_;

    // Data
    std::vector<Eigen::Vector2d> hippo_positions_;  
};
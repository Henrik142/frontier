#pragma once

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/float64.hpp>

#include <Eigen/Eigen>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <functional>
#include <memory>
#include <atomic>

#include "map.hpp"
#include "multi_map_manager.hpp"
#include "planning_maps.hpp"
#include "map_builder.hpp"
#include "raycast.hpp"
#include "visualization_utils.hpp"
#include "perception_utils.hpp"

#include "environment_map_msgs/msg/chunk_data.hpp"
#include "environment_map_msgs/msg/chunk_stamps.hpp"
#include "environment_map_msgs/msg/idx_list.hpp"

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using Odometry = nav_msgs::msg::Odometry;
using SyncPolicy = message_filters::sync_policies::ApproximateTime<PointCloud2, Odometry>;

// Builds and maintains the occupancy/ESDF map from incoming point clouds.
class MapUpdater : public rclcpp::Node
{
public:
    MapUpdater();
    ~MapUpdater();

    // Shared with the PathPlanningManager so it can query/plan on the same map instance.
    std::shared_ptr<Map> getMap() const;
    std::shared_ptr<std::atomic_bool> getMapReadyFlag() const;
    void getMapParams(Map::InitMapParams& map_params) const;
    void initPlanningMaps(const std::shared_ptr<PlanningMaps> & planning_maps);

private:
    void initParams();
    void pointCloudCallback(const PointCloud2::ConstSharedPtr & cloud_msg, 
        const Odometry::ConstSharedPtr & odometry_msg);
    void publishMap();
    void sendChunkStamps();
    void chunkStampsCallback(const environment_map_msgs::msg::ChunkStamps::SharedPtr msg);
    void chunkDataCallback(const environment_map_msgs::msg::ChunkData::SharedPtr msg);
    void processChunkBufferCallback();

    rclcpp::CallbackGroup::SharedPtr cb_group_main_;
    rclcpp::CallbackGroup::SharedPtr cb_group_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr send_stamp_timer_;
    rclcpp::TimerBase::SharedPtr process_chunk_buffer_timer_;

    // Subscribers and publishers
    message_filters::Subscriber<PointCloud2> subscription_pc_;
    message_filters::Subscriber<Odometry> subscription_odometry_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    rclcpp::Publisher<environment_map_msgs::msg::ChunkStamps>::SharedPtr publisher_chunk_stamps_;
    rclcpp::Subscription<environment_map_msgs::msg::ChunkStamps>::SharedPtr subscription_chunk_stamps_;
    rclcpp::Publisher<environment_map_msgs::msg::ChunkData>::SharedPtr publisher_chunk_data_;
    rclcpp::Subscription<environment_map_msgs::msg::ChunkData>::SharedPtr subscription_chunk_data_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_map_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_updated_box_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_distance_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_map_update_time_;

    // Params
    std::string vehicle_name_;
    PerceptionUtils::InitFOVParams fov_params_;
    Map::InitMapParams map_params_;
    int num_maps_;
    int chunk_size_;

    // Utils
    std::shared_ptr<MapBuilder> map_builder_;
    std::shared_ptr<VisualizationUtils> vis_utils_;
    std::shared_ptr<PerceptionUtils> percep_utils_;

    // Data
    std::shared_ptr<Map> map_;
    std::shared_ptr<MultiMapManager> multi_map_manager_;
    std::shared_ptr<PlanningMaps> planning_maps_;
    std::shared_ptr<std::atomic_bool> map_ready_ = std::make_shared<std::atomic_bool>(false);

    // Initialization flags
    bool init_box_discovered_ = false;

    // Time values
    double update_max_time_ = 0.0;
};

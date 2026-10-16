#pragma once

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <Eigen/Eigen>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <functional>
#include <memory>
#include <atomic>

#include "map.hpp"
#include "planning_maps.hpp"
#include "map_builder.hpp"
#include "raycast.hpp"
#include "visualization_utils.hpp"
#include "perception_utils.hpp"

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

    // Subscribers and publishers
    message_filters::Subscriber<PointCloud2> subscription_pc_;
    message_filters::Subscriber<Odometry> subscription_odometry_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_map_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_updated_box_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_distance_;

    // Params
    PerceptionUtils::InitFOVParams fov_params_;
    Map::InitMapParams map_params_;

    // Utils
    std::shared_ptr<MapBuilder> map_builder_;
    std::shared_ptr<VisualizationUtils> vis_utils_;
    std::shared_ptr<PerceptionUtils> percep_utils_;

    // Data
    std::shared_ptr<Map> map_;
    std::shared_ptr<PlanningMaps> planning_maps_;
    std::shared_ptr<std::atomic_bool> map_ready_ = std::make_shared<std::atomic_bool>(false);

    // Initialization flags
    bool init_box_discovered_ = false;

    // Time values
    double update_max_time_ = 0.0;
};

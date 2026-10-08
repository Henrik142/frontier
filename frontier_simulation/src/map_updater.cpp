#include "map_updater.hpp"

MapUpdater::MapUpdater() : Node("map_updater") {
    initParams();

    map_.reset(new Map());
    map_->initMap(map_params_);

    vis_utils_.reset(new VisualizationUtils());
    vis_utils_->initVisUtils(fov_params_);

    percep_utils_.reset(new PerceptionUtils());
    percep_utils_->initPercepUtils(fov_params_);

    publisher_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "occupancy_grid", rclcpp::QoS(1).transient_local());
    publisher_updated_box_ = this->create_publisher<visualization_msgs::msg::Marker>("sensor_updated_box", 10);
    publisher_distance_ = this->create_publisher<visualization_msgs::msg::Marker>("frontier/distance", 10);

    subscription_pc_.subscribe(this, "map_pointcloud");
    subscription_odometry_.subscribe(this, "odometry");

    // Create a synced callback that is called when there is a new point cloud and odometry message available
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), subscription_pc_, subscription_odometry_);
    sync_->registerCallback(std::bind(&MapUpdater::pointCloudCallback, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Node started");
}

MapUpdater::~MapUpdater() {
}

std::shared_ptr<Map> MapUpdater::getMap() const {
    return map_;
}

void MapUpdater::initParams() {
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

    // Map parameters
    declare_parameter<double>("map.xsize", 10.0);
    declare_parameter<double>("map.ysize", 10.0);
    declare_parameter<double>("map.resolution", 0.05);
    declare_parameter<double>("map.p_hit", 0.7);
    declare_parameter<double>("map.p_miss", 0.35);
    declare_parameter<double>("map.p_min", 0.12);
    declare_parameter<double>("map.p_max", 0.97);
    declare_parameter<double>("map.p_occ", 0.8);
    declare_parameter<double>("map.obstacles_inflation_radius", 0.1);
    declare_parameter<double>("map.unknown_inflation_radius", 0.1);
    declare_parameter<double>("map.esdf_inflation", 1.0);
    declare_parameter<double>("map.esdf_default_dist", 1.0);
    declare_parameter<bool>("map.esdf_optimistic", false);
    declare_parameter<bool>("map.esdf_signed_dist", false);

    map_params_.xsize = get_parameter("map.xsize").as_double();
    RCLCPP_INFO(get_logger(), "map.xsize=%f", map_params_.xsize);
    map_params_.ysize = get_parameter("map.ysize").as_double();
    RCLCPP_INFO(get_logger(), "map.ysize=%f", map_params_.ysize);
    map_params_.resolution = get_parameter("map.resolution").as_double();
    RCLCPP_INFO(get_logger(), "map.resolution=%f", map_params_.resolution);
    map_params_.p_hit = get_parameter("map.p_hit").as_double();
    RCLCPP_INFO(get_logger(), "map.p_hit=%f", map_params_.p_hit);
    map_params_.p_miss = get_parameter("map.p_miss").as_double();
    RCLCPP_INFO(get_logger(), "map.p_miss=%f", map_params_.p_miss);
    map_params_.p_min = get_parameter("map.p_min").as_double();
    RCLCPP_INFO(get_logger(), "map.p_min=%f", map_params_.p_min);
    map_params_.p_max = get_parameter("map.p_max").as_double();
    RCLCPP_INFO(get_logger(), "map.p_max=%f", map_params_.p_max);
    map_params_.p_occ = get_parameter("map.p_occ").as_double();
    RCLCPP_INFO(get_logger(), "map.p_occ=%f", map_params_.p_occ);
    map_params_.obstacles_inflation_radius = get_parameter("map.obstacles_inflation_radius").as_double();
    RCLCPP_INFO(get_logger(), "map.obstacles_inflation_radius=%f", map_params_.obstacles_inflation_radius);
    map_params_.unknown_inflation_radius = get_parameter("map.unknown_inflation_radius").as_double();
    RCLCPP_INFO(get_logger(), "map.unknown_inflation_radius=%f", map_params_.unknown_inflation_radius);

    map_params_.esdf_inflation = get_parameter("map.esdf_inflation").as_double();
    RCLCPP_INFO(get_logger(), "map.esdf_inflation=%f", map_params_.esdf_inflation);
    map_params_.esdf_default_dist = get_parameter("map.esdf_default_dist").as_double();
    RCLCPP_INFO(get_logger(), "map.esdf_default_dist=%f", map_params_.esdf_default_dist);
    map_params_.esdf_optimistic = get_parameter("map.esdf_optimistic").as_bool();
    RCLCPP_INFO(get_logger(), "map.esdf_optimistic=%s", map_params_.esdf_optimistic ? "true" : "false");
    map_params_.esdf_signed_dist = get_parameter("map.esdf_signed_dist").as_bool();
    RCLCPP_INFO(get_logger(), "map.esdf_signed_dist=%s", map_params_.esdf_signed_dist ? "true" : "false");

    map_params_.max_ray_length = fov_params_.max_dist;
    RCLCPP_INFO(get_logger(), "map.max_ray_length=%f", map_params_.max_ray_length);
}

void MapUpdater::pointCloudCallback(const PointCloud2::ConstSharedPtr & cloud_msg, 
        const Odometry::ConstSharedPtr & odometry_msg) {
    
    // Convert ROS message to PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    if (cloud->empty())
    {
      RCLCPP_WARN(this->get_logger(), "Received empty point cloud after transform, skipping");
      return;
    }

    // Get the camera position from the robot pose
    Eigen::Vector2d robot_position, camera_position;
    double yaw;

    robot_position << odometry_msg->pose.pose.position.x, odometry_msg->pose.pose.position.y;
    yaw = tf2::getYaw(odometry_msg->pose.pose.orientation);

    percep_utils_->getCameraPosFromRobotPose(camera_position, robot_position, yaw);

    // Update the map with the new point cloud
    RCLCPP_INFO(this->get_logger(), "Starting Map update.");
    
    if (map_->getOccupancy(camera_position) == Map::FREE) {
        map_->inputPointCloud(*cloud, cloud->size(), camera_position);
    } else {
        RCLCPP_WARN(this->get_logger(), "Camera position is not in free space, skipping point cloud input.");
        return;
    }

    // Inflate obstacles and update the ESDF
    map_->inflateObstacles();
    //map_->updateESDF2d();

    // Publish the map
    publishMap();

    // Create and publish the updated box markers
    visualization_msgs::msg::Marker marker_updated_box;
    marker_updated_box.header.frame_id = "map";
    marker_updated_box.header.stamp = this->now();

    Eigen::Vector2i bmin, bmax;
    map_->getUpdatedBox(bmin, bmax);
    vis_utils_->drawBox(marker_updated_box, bmin, bmax, *map_);
    publisher_updated_box_->publish(marker_updated_box);

    // Create and publish the distance field markers
    visualization_msgs::msg::Marker marker_distance;
    marker_distance.header.frame_id = "map";
    marker_distance.header.stamp = this->now();

    vis_utils_->drawDistanceField(marker_distance, *map_, 1.0);
    publisher_distance_->publish(marker_distance);

    RCLCPP_INFO(this->get_logger(), "Map updated and published.");
}

void MapUpdater::publishMap() {
    nav_msgs::msg::OccupancyGrid grid_msg;
    grid_msg.header.stamp = this->now();
    grid_msg.header.frame_id = "map";

    vis_utils_->drawMap(grid_msg, *map_, false);

    publisher_map_->publish(grid_msg);
}

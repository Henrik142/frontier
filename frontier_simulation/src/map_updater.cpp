#include "map_updater.hpp"

MapUpdater::MapUpdater() : Node("map_updater") {
    initParams();

    map_.reset(new Map());
    map_->initMap(map_params_);
    map_builder_.reset(new MapBuilder(map_.get()));

    int hippo_id = std::stoi(vehicle_name_.substr(vehicle_name_.size() - 2));
    RCLCPP_INFO(this->get_logger(), "hippo_id=%d", hippo_id);

    multi_map_manager_.reset(new MultiMapManager());
    multi_map_manager_->init(hippo_id, num_maps_, chunk_size_);
    multi_map_manager_->setMap(map_);

    /*What do we need to call?

    -------- cb_group_main_ ----------

    updateMapChunk
    - call after inputPointCloud
    - Forms new chunks from observed voxels
    - operates on: adr_buffer_, multi_map_chunks_

    processChunkBuffer
    - call at fixed frequency
    - Processes messages from the buffer to update this robot's map
    - operates on: chunk_buffer_, buffer_map_, multi_map_chunks_, map_, chunk_boxes_
                      
    sendStamp
    - call at fixed frequency
    - Tells other robots which chunks this robot knows about
    - operates on: multi_map_chunks_

    receiveStamp
    - callback when receiving stamp message (subscription)
    - Checks which chunks other robots know about, and sends chunks that other robots don't know about yet
    - operates on: multi_map_chunks_, map_
    - only operates on the map_ to read the occupancy information

    receiveChunk
    - callback when receiving chunk message (subscription)
    - Adds chunk messages to the internal buffer for later processing
    - operates on: chunk_buffer_, buffer_map_
    
    */

    vis_utils_.reset(new VisualizationUtils());
    vis_utils_->initVisUtils(fov_params_);

    percep_utils_.reset(new PerceptionUtils());
    percep_utils_->initPercepUtils(fov_params_);

    publisher_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "occupancy_grid", rclcpp::QoS(1).transient_local());
    publisher_updated_box_ = this->create_publisher<visualization_msgs::msg::Marker>("sensor_updated_box", 10);
    publisher_distance_ = this->create_publisher<visualization_msgs::msg::Marker>("frontier/distance", 10);
    publisher_map_update_time_ = this->create_publisher<std_msgs::msg::Float64>("frontier/times/map_update", 10);

    publisher_chunk_stamps_ = this->create_publisher<environment_map_msgs::msg::ChunkStamps>(
            "/multi_map_manager/chunk_stamps", rclcpp::QoS(1).transient_local());
    publisher_chunk_data_ = this->create_publisher<environment_map_msgs::msg::ChunkData>(
            "/multi_map_manager/chunk_data", rclcpp::QoS(1).transient_local());

    // Initialize two mutually exclusive callback groups that enable concurrent callbacks
    cb_group_main_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_group_pub_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Create point cloud and odometry subscriptions
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = cb_group_main_;

    subscription_pc_.subscribe(this, "map_pointcloud", rmw_qos_profile_default, sub_options);
    subscription_odometry_.subscribe(this, "odometry", rmw_qos_profile_default, sub_options);

    subscription_chunk_stamps_ = this->create_subscription<environment_map_msgs::msg::ChunkStamps>(
        "/multi_map_manager/chunk_stamps",
        rclcpp::QoS(1).transient_local(),
        std::bind(&MapUpdater::chunkStampsCallback, this, std::placeholders::_1),
        sub_options);

    subscription_chunk_data_ = this->create_subscription<environment_map_msgs::msg::ChunkData>(
        "/multi_map_manager/chunk_data",
        rclcpp::QoS(1).transient_local(),
        std::bind(&MapUpdater::chunkDataCallback, this, std::placeholders::_1),
        sub_options);

    // Create a synced callback that is called when there is a new point cloud and odometry message available
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), subscription_pc_, subscription_odometry_);
    sync_->registerCallback(std::bind(&MapUpdater::pointCloudCallback, this,
        std::placeholders::_1, std::placeholders::_2));

    // Create a timer for publishing the map to Rviz
    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&MapUpdater::publishMap, this),
        cb_group_pub_);

    // Create a timer for sending chunk stamps
    send_stamp_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&MapUpdater::sendChunkStamps, this),
        cb_group_main_);

    // Create a timer for processing the chunk buffer
    process_chunk_buffer_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&MapUpdater::processChunkBufferCallback, this),
        cb_group_main_);

    RCLCPP_INFO(this->get_logger(), "Node started");
}

MapUpdater::~MapUpdater() {
}

std::shared_ptr<Map> MapUpdater::getMap() const {
    return map_;
}

std::shared_ptr<std::atomic_bool> MapUpdater::getMapReadyFlag() const {
    return map_ready_;
}

void MapUpdater::getMapParams(Map::InitMapParams& map_params) const {
    map_params = map_params_;
}

void MapUpdater::initPlanningMaps(const std::shared_ptr<PlanningMaps> & planning_maps) {
    planning_maps_ = planning_maps;
}

void MapUpdater::initParams() {
    vehicle_name_ = declare_parameter<std::string>("vehicle_name", "");

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
    declare_parameter<int>("map.num_maps", 2);
    declare_parameter<int>("map.chunk_size", 200);

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

    num_maps_ = get_parameter("map.num_maps").as_int();
    RCLCPP_INFO(get_logger(), "map.num_maps=%d", num_maps_);
    chunk_size_ = get_parameter("map.chunk_size").as_int();
    RCLCPP_INFO(get_logger(), "map.chunk_size=%d", chunk_size_);
}

void MapUpdater::pointCloudCallback(const PointCloud2::ConstSharedPtr & cloud_msg, 
        const Odometry::ConstSharedPtr & odometry_msg) {
    double t1 = this->now().seconds();

    Eigen::Vector2d robot_position;
    double yaw;
    robot_position << odometry_msg->pose.pose.position.x, odometry_msg->pose.pose.position.y;
    yaw = tf2::getYaw(odometry_msg->pose.pose.orientation);

    // "discover" a box around the initial position
    if (!init_box_discovered_) {
        Eigen::Vector2d box_mid_pos;
        box_mid_pos.x() = robot_position.x();
        box_mid_pos.y() = robot_position.y();

        double side_length = 0.5;
        Eigen::Vector2d box_min_pos = box_mid_pos - 0.5 * Eigen::Vector2d::Ones() * side_length;
        Eigen::Vector2d box_max_pos = box_mid_pos + 0.5 * Eigen::Vector2d::Ones() * side_length;
        map_builder_->setBox(box_min_pos, box_max_pos, Map::FREE);
        // map_->inflateObstacles();

        RCLCPP_INFO(this->get_logger(), "Initialization box set.");
        init_box_discovered_ = true;
    }
    
    // Convert ROS message to PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    if (cloud->empty())
    {
      RCLCPP_WARN(this->get_logger(), "Received empty point cloud after transform, skipping");
      return;
    }

    // Get the camera position from the robot pose
    Eigen::Vector2d camera_position;
    percep_utils_->getCameraPosFromRobotPose(camera_position, robot_position, yaw);

    // Update the map with the new point cloud
    RCLCPP_INFO(this->get_logger(), "Starting Map update.");
    // RCLCPP_INFO(this->get_logger(), "Camera position: x=%f, y=%f", camera_position.x(), camera_position.y());
    
    if (map_->getOccupancy(camera_position) == Map::FREE) {
        // Check if the updated box needs to be reset. Each time the main loop of the path planning manager
        // runs, this is true. So basically this means: If the path planning manager main loop ran at least once
        // since the last point cloud update, then reset the updated box. Otherwise, don't reset it.
        bool reset_updated_box = planning_maps_->getResetUpdatedBox(false);
        
        std::vector<int> updated_adrs;
        map_->inputPointCloud(*cloud, cloud->size(), camera_position, reset_updated_box, updated_adrs);
        
        std::vector<uint32_t> unsigned_adrs(updated_adrs.begin(), updated_adrs.end());
        multi_map_manager_->updateMapChunk(unsigned_adrs);
    } else {
        RCLCPP_WARN(this->get_logger(), "Camera position is not in free space, skipping point cloud input.");
        return;
    }

    // Inflate obstacles
    map_->inflateObstacles();
    // RCLCPP_INFO(this->get_logger(), "Obstacles inflated.");

    // Update the ESDF
    //map_->updateESDF2d();

    // Store the map data in planning_maps_ so it is available for the pathplanning module
    // as well as for publishing to Rviz
    if (planning_maps_->publish(*map_)) {
        RCLCPP_INFO(this->get_logger(), "Map written to planning maps.");
    } else {
        RCLCPP_WARN(this->get_logger(), "Failed to write map to planning maps.");
        return;
    }
    map_ready_->store(true, std::memory_order_release);

    double t2 = this->now().seconds();
    if (t2 - t1 > update_max_time_) {
        update_max_time_ = t2 - t1;
    }
    std_msgs::msg::Float64 update_time_msg;
    update_time_msg.data = t2 - t1;
    publisher_map_update_time_->publish(update_time_msg);
    RCLCPP_INFO(this->get_logger(), "Map update completed in %f seconds (max time: %f)", t2 - t1, update_max_time_);
}

void MapUpdater::publishMap() {
    // This is purely for visualization in rviz

    if (!planning_maps_ || !map_ready_ || !map_ready_->load(std::memory_order_acquire)) {
        return;
    }

    double t1 = this->now().seconds();

    std::shared_ptr<const Map> map_snapshot = planning_maps_->acquire();

    nav_msgs::msg::OccupancyGrid grid_msg;
    grid_msg.header.stamp = this->now();
    grid_msg.header.frame_id = "map";

    bool show_inflation = false;
    vis_utils_->drawMap(grid_msg, *map_snapshot, show_inflation);

    publisher_map_->publish(grid_msg);

    // Create and publish the updated box markers
    visualization_msgs::msg::Marker marker_updated_box;
    marker_updated_box.header.frame_id = "map";
    marker_updated_box.header.stamp = this->now();

    Eigen::Vector2i bmin, bmax;
    map_snapshot->getUpdatedBox(bmin, bmax);
    vis_utils_->drawBox(marker_updated_box, bmin, bmax, *map_snapshot);
    publisher_updated_box_->publish(marker_updated_box);

    /*
    // Create and publish the distance field markers
    visualization_msgs::msg::Marker marker_distance;
    marker_distance.header.frame_id = "map";
    marker_distance.header.stamp = this->now();

    vis_utils_->drawDistanceField(marker_distance, *map_snapshot, 1.0);
    publisher_distance_->publish(marker_distance);
    */

    double t2 = this->now().seconds();
    RCLCPP_INFO(this->get_logger(), "Map publishing completed in %f seconds", t2 - t1);
}

void MapUpdater::sendChunkStamps() {
    environment_map_msgs::msg::ChunkStamps chunk_stamps_msg;
    double time = this->now().seconds();
    multi_map_manager_->sendStamp(time, chunk_stamps_msg);
    publisher_chunk_stamps_->publish(chunk_stamps_msg);
}

void MapUpdater::chunkStampsCallback(const environment_map_msgs::msg::ChunkStamps::SharedPtr msg) {
    std::vector<environment_map_msgs::msg::ChunkData> chunk_data_msgs;
    multi_map_manager_->receiveStamp(*msg, chunk_data_msgs);
    for (const auto & chunk_data_msg : chunk_data_msgs) {
        publisher_chunk_data_->publish(chunk_data_msg);
    }
}

void MapUpdater::chunkDataCallback(const environment_map_msgs::msg::ChunkData::SharedPtr msg) {
    multi_map_manager_->receiveChunk(*msg);
}

void MapUpdater::processChunkBufferCallback() {
    multi_map_manager_->processChunkBuffer(map_params_.obstacles_inflation_radius);
}
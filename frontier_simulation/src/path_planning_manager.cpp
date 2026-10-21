#include "path_planning_manager.hpp"

PathPlanningManager::PathPlanningManager(
        const std::shared_ptr<PlanningMaps> & planning_maps,
        const std::shared_ptr<std::atomic_bool> & map_ready)
        : Node("path_planning_manager") {
    // Initialize the map
    planning_maps_ = planning_maps;
    most_recent_map_ = planning_maps_->acquire(true);
    map_ready_ = map_ready;

    // Initialize parameters
    initParams();

    // TODO: find the actual values of these parameters for the cost function
    ViewNode::vm_ = vm_;        // maximum velocity
    ViewNode::yd_ = yd_;        // maximum yaw rate
    ViewNode::w_y = w_y_;          // yaw change weight
    ViewNode::w_dir_ = w_dir_;     // motion consistency weight

    // Initialize utils
    vis_utils_.reset(new VisualizationUtils());
    vis_utils_->initVisUtils(fov_params_);

    frontier_finder_ = std::make_shared<FrontierFinder>();
    frontier_finder_->initFrontierFinder(most_recent_map_, fov_params_, frontier_params_);

    ViewNode::astar_.reset(new Astar);
    ViewNode::astar_->init(path_astar_params_, most_recent_map_);
    ViewNode::map_ = most_recent_map_;

    double resolution = most_recent_map_->getResolution();
    Eigen::Vector2d origin(most_recent_map_->getOriginX(), most_recent_map_->getOriginY());
    ViewNode::caster_.reset(new RayCaster);
    ViewNode::caster_->setParams(resolution, origin);

    // Parameter file for LKH
    namespace fs = std::filesystem;
    fs::path dir = fs::temp_directory_path() / ("lkh_" + vehicle_name_);
    fs::create_directories(dir);
    data_dir_ = dir.string();
    {
        std::ofstream par_file(data_dir_ + "/single.par");
        par_file << "PROBLEM_FILE = " << data_dir_ << "/single.tsp\n"
                << "MOVE_TYPE = 5\n"
                << "PATCHING_C = 3\n"
                << "PATCHING_A = 2\n"
                << "RUNS = 10\n"
                << "OUTPUT_TOUR_FILE = " << data_dir_ << "/single.txt\n";
    }

    // TF buffer + listener: subscribes to /tf and /tf_static in the background
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Initialize two mutually exclusive callback groups that enable concurrent callbacks
    cb_group_path_planning_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_group_odometry_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Initialize publishers and subscribers
    publisher_global_path_ = this->create_publisher<visualization_msgs::msg::Marker>("frontier/global_path", 10);
    publisher_refined_views_ = this->create_publisher<visualization_msgs::msg::Marker>("frontier/refined_views", 10);
    publisher_next_viewpoint_ = this->create_publisher<visualization_msgs::msg::Marker>("frontier/next_viewpoint", 10);
    publisher_frontiers_ = this->create_publisher<visualization_msgs::msg::Marker>("frontier/cells", 10);
    publisher_viewpoints_ = this->create_publisher<visualization_msgs::msg::Marker>("frontier/viewpoints", 10);
    publisher_cluster_centroids_ = this->create_publisher<visualization_msgs::msg::Marker>("frontier/cluster_centroids", 10);
    publisher_fov_ = this->create_publisher<visualization_msgs::msg::Marker>("frontier/viewpoints_fov", 10);
    publisher_velocity_ = this->create_publisher<visualization_msgs::msg::Marker>("velocity", 10);
    publisher_string_ = this->create_publisher<std_msgs::msg::String>("frontier/status", 10);
    publisher_pos_target_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("frontier/next_pos", 10);
    publisher_yaw_target_ = this->create_publisher<std_msgs::msg::Float64>("frontier/next_yaw", 10);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = cb_group_odometry_;

    subscription_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry",
        rclcpp::SensorDataQoS(),   // sensor data QoS: best-effort, suited for high-rate streams
        std::bind(&PathPlanningManager::odometryCallback, this, std::placeholders::_1),
        sub_options);

    // Create timer for the path planning initialization
    path_planning_init_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(update_rate_ms_),
        std::bind(&PathPlanningManager::pathPlanningInit, this));
    
    // Create timer that plans a tour over the frontier clusters at a fixed frequency
    path_planning_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(update_rate_ms_),
        std::bind(&PathPlanningManager::pathPlanningCallback, this),
        cb_group_path_planning_);

    RCLCPP_INFO(this->get_logger(), "Node started");
}

PathPlanningManager::~PathPlanningManager() {
}

void PathPlanningManager::initParams() {
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

    // Frontier parameters
    declare_parameter<double>("frontier.update_inflation_size", 0.2);
    declare_parameter<double>("frontier.cluster_max_radius", 0.5);
    declare_parameter<int>("frontier.candidate_rnum", 8);
    declare_parameter<double>("frontier.candidate_rmin", 0.1);
    declare_parameter<double>("frontier.candidate_rmax", 0.4);
    declare_parameter<double>("frontier.candidate_dphi", 22.5);
    declare_parameter<double>("frontier.candidate_theta", 180.0);
    declare_parameter<int>("frontier.candidate_theta_num", 7);
    declare_parameter<double>("frontier.min_candidate_dist", 0.1);
    declare_parameter<double>("frontier.min_candidate_yaw_diff", 5.0);
    declare_parameter<int>("frontier.min_visib_num", 2);

    frontier_params_.update_inflation_size = get_parameter("frontier.update_inflation_size").as_double();
    RCLCPP_INFO(get_logger(), "frontier.update_inflation_size=%f", frontier_params_.update_inflation_size);
    frontier_params_.cluster_max_radius = get_parameter("frontier.cluster_max_radius").as_double();
    RCLCPP_INFO(get_logger(), "frontier.cluster_max_radius=%f", frontier_params_.cluster_max_radius);
    frontier_params_.candidate_rnum = get_parameter("frontier.candidate_rnum").as_int();
    RCLCPP_INFO(get_logger(), "frontier.candidate_rnum=%d", frontier_params_.candidate_rnum);
    frontier_params_.candidate_rmin = get_parameter("frontier.candidate_rmin").as_double();
    RCLCPP_INFO(get_logger(), "frontier.candidate_rmin=%f", frontier_params_.candidate_rmin);
    frontier_params_.candidate_rmax = get_parameter("frontier.candidate_rmax").as_double();
    RCLCPP_INFO(get_logger(), "frontier.candidate_rmax=%f", frontier_params_.candidate_rmax);
    frontier_params_.candidate_dphi = get_parameter("frontier.candidate_dphi").as_double();
    RCLCPP_INFO(get_logger(), "frontier.candidate_dphi=%f", frontier_params_.candidate_dphi);
    frontier_params_.candidate_theta = get_parameter("frontier.candidate_theta").as_double();
    RCLCPP_INFO(get_logger(), "frontier.candidate_theta=%f", frontier_params_.candidate_theta);
    frontier_params_.candidate_theta_num = get_parameter("frontier.candidate_theta_num").as_int();
    RCLCPP_INFO(get_logger(), "frontier.candidate_theta_num=%d", frontier_params_.candidate_theta_num);
    frontier_params_.min_candidate_dist = get_parameter("frontier.min_candidate_dist").as_double();
    RCLCPP_INFO(get_logger(), "frontier.min_candidate_dist=%f", frontier_params_.min_candidate_dist);
    frontier_params_.min_candidate_yaw_diff = get_parameter("frontier.min_candidate_yaw_diff").as_double();
    RCLCPP_INFO(get_logger(), "frontier.min_candidate_yaw_diff=%f", frontier_params_.min_candidate_yaw_diff);
    frontier_params_.min_visib_num = get_parameter("frontier.min_visib_num").as_int();
    RCLCPP_INFO(get_logger(), "frontier.min_visib_num=%d", frontier_params_.min_visib_num);

    // Convert degrees to radians
    frontier_params_.candidate_dphi = frontier_params_.candidate_dphi * M_PI / 180.0;
    frontier_params_.candidate_theta = frontier_params_.candidate_theta * M_PI / 180.0;
    frontier_params_.min_candidate_yaw_diff = frontier_params_.min_candidate_yaw_diff * M_PI / 180.0;

    // Path planning parameters
    declare_parameter<int>("path_planning.update_rate_ms", 200);
    declare_parameter<bool>("path_planning.global_tour_greedy", true);
    declare_parameter<bool>("path_planning.refine_local", true);
    declare_parameter<int>("path_planning.refine_num_clusters", 2);
    declare_parameter<double>("path_planning.refine_radius", 1.0);
    declare_parameter<int>("path_planning.refine_num_viewpoints", 20);
    declare_parameter<int>("path_planning.refine_num_viewpoints_first", 30);
    declare_parameter<double>("path_planning.refine_max_decay", 0.5);
    declare_parameter<double>("path_planning.vm", 1.0);
    declare_parameter<double>("path_planning.yd", 0.7);
    declare_parameter<double>("path_planning.w_y", 1.0);
    declare_parameter<double>("path_planning.w_dir", 1.0);
    declare_parameter<double>("path_planning.astar.resolution", 0.3);
    declare_parameter<double>("path_planning.astar.lambda_heu", 10.0);
    declare_parameter<double>("path_planning.astar.max_search_time_global", 0.001);
    declare_parameter<double>("path_planning.astar.max_search_time_local", 0.01);
    declare_parameter<int>("path_planning.astar.allocate_num", 100000);

    update_rate_ms_ = get_parameter("path_planning.update_rate_ms").as_int();
    RCLCPP_INFO(get_logger(), "path_planning.update_rate_ms=%d", update_rate_ms_);
    global_tour_greedy_ = get_parameter("path_planning.global_tour_greedy").as_bool();
    RCLCPP_INFO(get_logger(), "path_planning.global_tour_greedy=%s", global_tour_greedy_ ? "true" : "false");
    refine_local_ = get_parameter("path_planning.refine_local").as_bool();
    RCLCPP_INFO(get_logger(), "path_planning.refine_local=%s", refine_local_ ? "true" : "false");
    refine_num_clusters_ = get_parameter("path_planning.refine_num_clusters").as_int();
    RCLCPP_INFO(get_logger(), "path_planning.refine_num_clusters=%d", refine_num_clusters_);
    refine_radius_ = get_parameter("path_planning.refine_radius").as_double();
    RCLCPP_INFO(get_logger(), "path_planning.refine_radius=%f", refine_radius_);
    refine_num_viewpoints_ = get_parameter("path_planning.refine_num_viewpoints").as_int();
    RCLCPP_INFO(get_logger(), "path_planning.refine_num_viewpoints=%d", refine_num_viewpoints_);
    refine_num_viewpoints_first_ = get_parameter("path_planning.refine_num_viewpoints_first").as_int();
    RCLCPP_INFO(get_logger(), "path_planning.refine_num_viewpoints_first=%d", refine_num_viewpoints_first_);
    refine_max_decay_ = get_parameter("path_planning.refine_max_decay").as_double();
    RCLCPP_INFO(get_logger(), "path_planning.refine_max_decay=%f", refine_max_decay_);
    vm_ = get_parameter("path_planning.vm").as_double();
    RCLCPP_INFO(get_logger(), "path_planning.vm=%f", vm_);
    yd_ = get_parameter("path_planning.yd").as_double();
    RCLCPP_INFO(get_logger(), "path_planning.yd=%f", yd_);
    w_y_ = get_parameter("path_planning.w_y").as_double();
    RCLCPP_INFO(get_logger(), "path_planning.w_y=%f", w_y_);
    w_dir_ = get_parameter("path_planning.w_dir").as_double();
    RCLCPP_INFO(get_logger(), "path_planning.w_dir=%f", w_dir_);
    path_astar_params_.resolution = get_parameter("path_planning.astar.resolution").as_double();
    RCLCPP_INFO(get_logger(), "path_planning.astar.resolution=%f", path_astar_params_.resolution);
    path_astar_params_.lambda_heu = get_parameter("path_planning.astar.lambda_heu").as_double();
    RCLCPP_INFO(get_logger(), "path_planning.astar.lambda_heu=%f", path_astar_params_.lambda_heu);

    max_search_time_global_ = get_parameter("path_planning.astar.max_search_time_global").as_double();
    RCLCPP_INFO(get_logger(), "path_planning.astar.max_search_time_global=%f", max_search_time_global_);
    max_search_time_local_ = get_parameter("path_planning.astar.max_search_time_local").as_double();
    RCLCPP_INFO(get_logger(), "path_planning.astar.max_search_time_local=%f", max_search_time_local_);

    path_astar_params_.max_search_time = max_search_time_global_;

    path_astar_params_.allocate_num = get_parameter("path_planning.astar.allocate_num").as_int();
    RCLCPP_INFO(get_logger(), "path_planning.astar.allocate_num=%d", path_astar_params_.allocate_num);
}

void PathPlanningManager::pathPlanningInit() {
    if (!path_planning_initialized_) {
        publishEmptyMarkers();
    }

    // Wait for the first odometry message and the map initialization
    if (!odometry_received_ || !map_ready_->load(std::memory_order_acquire))
        return;

    // Complete one 360-degree rotation before starting the main path planning loop
    if (!init_rotation_started_) {
        hippo_yaw_start_ = hippo_yaw_;
        RCLCPP_INFO(this->get_logger(), "Starting initial rotation.");
        init_rotation_started_ = true;
    } else if (!init_rotation_finished_) {
        double wrapped_yaw = wrapYaw(hippo_yaw_ - hippo_yaw_start_);
        double target_yaw;
        if (wrapped_yaw < 3* M_PI / 2 && wrapped_yaw >= - M_PI / 4) {
            target_yaw = hippo_yaw_ + M_PI / 4;
            if (wrapped_yaw >= M_PI) {
                init_rotation_halfway_ = true;
            }
        }

        if (init_rotation_halfway_ && wrapped_yaw < M_PI / 4) {
            target_yaw = hippo_yaw_start_;
        }

        if (init_rotation_halfway_ && fabs(wrapped_yaw) < 1e-3) {
            init_rotation_finished_ = true;
            path_planning_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Finished initial rotation.");
            path_planning_init_timer_->cancel();
            path_planning_init_timer_.reset();
            return;
        }

        geometry_msgs::msg::Vector3Stamped target_pos_msg;
        target_pos_msg.header.stamp = this->now();
        target_pos_msg.header.frame_id = "map";
        target_pos_msg.vector.x = hippo_position_.x();
        target_pos_msg.vector.y = hippo_position_.y();
        target_pos_msg.vector.z = 0.0;
        publisher_pos_target_->publish(target_pos_msg);

        std_msgs::msg::Float64 target_yaw_msg;
        target_yaw_msg.data = target_yaw;
        publisher_yaw_target_->publish(target_yaw_msg);
    }
}

int PathPlanningManager::pathPlanningCallback() {
    // only execute if path planning has been initialized
    if (!path_planning_initialized_)
        return WAITING_FOR_INIT;

    RCLCPP_INFO(this->get_logger(), "Starting pathplanning...");

    double t1 = this->now().seconds();

    most_recent_map_ = planning_maps_->acquire(true);
    frontier_finder_->setMap(most_recent_map_);
    ViewNode::map_ = most_recent_map_;
    ViewNode::astar_->setMap(most_recent_map_);

    path_astar_params_.max_search_time = max_search_time_global_;
    ViewNode::astar_->setParams(path_astar_params_);

    // find frontiers, sample viewpoints
    frontier_finder_->findFrontiers();
    frontier_finder_->computeClustersToVisit();
    
    if (frontier_finder_->getFrontierClusters().empty()) {
        RCLCPP_INFO(this->get_logger(), "No frontier clusters available.");
        publishEmptyMarkers();
        return NO_FRONTIER;
    }

    std::vector<int> removed_clusters = frontier_finder_->getRemovedIds();
    std::string removed_info = "Removed cluster IDs: ";
    for (int removed_id : removed_clusters) {
        std::ostringstream oss;
        oss << removed_id;
        if (removed_id != removed_clusters.back()) oss << ",";
        removed_info += oss.str();
    }
    RCLCPP_INFO(this->get_logger(), "%s", removed_info.c_str());

    double t_intermediate = this->now().seconds();
    RCLCPP_INFO(this->get_logger(), "New frontier clusters computed in %f seconds.",
        t_intermediate - t1);

    // Get cost matrix for current state and clusters
    Eigen::MatrixXd cost_mat;
    RCLCPP_INFO(this->get_logger(), "Updating frontier cost matrix");
    frontier_finder_->updateFrontierCostMatrix();
    frontier_finder_->getFullCostMatrix(hippo_position_, hippo_velocity_, hippo_yaw_, cost_mat);

    RCLCPP_INFO(this->get_logger(), "New cost matrix computed in %f seconds.",
        this->now().seconds() - t_intermediate);

    // Make a cluster_indexer to access the frontier list easier
    std::list<FrontierCluster> frontier_clusters = frontier_finder_->getFrontierClusters();
    std::vector<std::list<FrontierCluster>::iterator> cluster_indexer;
    for (auto it = frontier_clusters.begin(); it != frontier_clusters.end(); ++it)
        cluster_indexer.push_back(it);

    // Publish the frontiers
    publishFrontierStatus(frontier_clusters);

    // Print the cost matrix
    RCLCPP_INFO(this->get_logger(), "Cost matrix is of dimension: [%zux%zu]",
                cost_mat.rows(), cost_mat.cols());
    /*
    for (int i = 0; i < cost_mat.rows(); ++i) {
        std::ostringstream oss;
        for (int j = 0; j < cost_mat.cols(); ++j) {
            if (j > 0) oss << ",";
            oss << cost_mat(i, j);
        }
        std::string result = oss.str();
        RCLCPP_INFO(this->get_logger(), "Row %d: [%s]", i, result.c_str());
    }
    */

    t_intermediate = this->now().seconds();
    if (t_intermediate - t1 > frontier_max_time_) frontier_max_time_ = t_intermediate - t1;
    RCLCPP_INFO(this->get_logger(), "Frontier update completed in %f seconds (max time: %f seconds)",
                t_intermediate - t1, frontier_max_time_);

    // Compute the global tour (i.e. the order in which to visit the clusters)
    std::vector<int> tour;
    findGlobalTour(tour, cost_mat);

    // Print the global tour
    std::ostringstream tour_oss;
    for (size_t i = 0; i < tour.size(); ++i) {
        if (i > 0) tour_oss << ",";
        tour_oss << tour[i];
    }
    std::string tour_result = tour_oss.str();
    RCLCPP_INFO(this->get_logger(), "Tour: [%s]", tour_result.c_str());

    if (tour.size() < 1)
        return NO_TOUR;

    // Get the path connecting the viewpoints in the tour
    path_astar_params_.max_search_time = max_search_time_local_;
    ViewNode::astar_->setParams(path_astar_params_);

    std::vector<Eigen::Vector2d> path;
    frontier_finder_->getPathForTour(hippo_position_, tour, path);
    RCLCPP_INFO(this->get_logger(), "Global path has %zu points.", path.size());

    RCLCPP_INFO(this->get_logger(), "Global path computation completed in %f seconds",
                this->now().seconds() - t_intermediate);
    t_intermediate = this->now().seconds();

    // Find the target position and yaw for trajectory planning
    Eigen::Vector2d next_pos;
    double next_yaw;

    // Refine the local tour if the flag is enabled
    std::vector<Eigen::Vector2d> refined_points;
    std::vector<double> refined_yaws;
    if (refine_local_) {
        // Vector that holds the IDs of the clusters that are considered for refinement
        std::vector<int> refined_ids;

        // Number of clusters to consider in the refinement
        int k_num = std::min(refine_num_clusters_, static_cast<int>(tour.size()));

        // Store the IDs of the first k clusters in the tour.
        // Additionally, check the distance from the current hippo position and stop
        // adding clusters if the cluster centroid is too far away.
        for (int i = 0; i < k_num; ++i) {
            if ((cluster_indexer[tour[i]]->average_pos_ - hippo_position_).norm()
                    > refine_radius_) break;
            refined_ids.push_back(tour[i]);
        }

        // Check if there are enough clusters for refinement to proceed
        if (!refined_ids.empty()) {
            RCLCPP_INFO(this->get_logger(), "Number of clusters in the refinement is %d",
                static_cast<int>(refined_ids.size()));


            // Define vector that specifies the maximum number of viewpoints for each cluster in the
            // refinement
            std::vector<int> refine_num_viewpoints_vec(refined_ids.size(), refine_num_viewpoints_);
            refine_num_viewpoints_vec[0] = refine_num_viewpoints_first_;

            // Get top viewpoints for the next K frontiers. These are the viewpoints that
            // will be considered for refinement.
            // For the first cluster, all viewpoints above the visibility threshold
            // are considered (refine_max_decay_).
            // For all subsequent clusters, additionally to checking the threshold,
            // the maximum number of viewpoints is also considered (refine_num_viewpoints_).
            std::vector<std::vector<Eigen::Vector2d>> n_points;
            std::vector<std::vector<double>> n_yaws;
            frontier_finder_->getViewpointsInfo(hippo_position_,
                                                hippo_yaw_,
                                                refined_ids,
                                                refine_num_viewpoints_vec,
                                                refine_max_decay_,
                                                n_points,
                                                n_yaws);

            // Calculate the refined positions and yaws
            refineLocalTour(hippo_position_,
                            hippo_velocity_,
                            hippo_yaw_,
                            n_points,
                            n_yaws,
                            refined_points,
                            refined_yaws);

            next_pos = refined_points.front();
            next_yaw = refined_yaws.front();

            RCLCPP_INFO(this->get_logger(), "Tour refinement completed in %f seconds",
                this->now().seconds() - t_intermediate);
        } else {
            // If no clusters are available for refinement,
            // fall back to the top viewpoint of the first cluster
            next_pos = cluster_indexer[tour.front()]->viewpoints_.front().robot_pos_;
            next_yaw = cluster_indexer[tour.front()]->viewpoints_.front().yaw_;

            // TODO: Maybe publish empty refined viewpoints marker
        }
    } else {        
        // Simply pick the top viewpoint of the first cluster in the tour
        next_pos = cluster_indexer[tour.front()]->viewpoints_.front().robot_pos_;
        next_yaw = cluster_indexer[tour.front()]->viewpoints_.front().yaw_;
    }

    /*
    Publish everything
    */

    // Publish the target position of the next viewpoint
    geometry_msgs::msg::Vector3Stamped target_pos_msg;
    target_pos_msg.header.stamp = this->now();
    target_pos_msg.header.frame_id = "map";
    target_pos_msg.vector.x = next_pos.x();
    target_pos_msg.vector.y = next_pos.y();
    target_pos_msg.vector.z = 0.0;
    publisher_pos_target_->publish(target_pos_msg);

    // Publish the target yaw of the next viewpoint
    std_msgs::msg::Float64 target_yaw_msg;
    target_yaw_msg.data = next_yaw;
    publisher_yaw_target_->publish(target_yaw_msg);

    // Publish frontier markers for rviz
    publishPathplanningStatus(path, next_pos, next_yaw, refined_points);

    double t2 = this->now().seconds();
    if (t2 - t1 > pathplanning_max_time_) {
        pathplanning_max_time_ = t2 - t1;
    }
    RCLCPP_INFO(this->get_logger(), "Path planning completed in %f seconds (max time: %f seconds)",
        t2 - t1, pathplanning_max_time_);

    return SUCCEED;
}

void PathPlanningManager::findGlobalTour(std::vector<int> & tour,
                                    const Eigen::MatrixXd & cost_mat) {
    tour.clear();
    const int dimension = cost_mat.rows();
    
    if (dimension <= 2) {
        // Only one cluster available, no need to compute a tour
        tour.push_back(0);
        
    } else if (global_tour_greedy_) {
        // Find "greedy" global tour that passes all clusters
        // by first going to the cluster with the lowest cost, then going to the
        // next cluster that has the lowest cost from the first one, and so on.
        tour.push_back(0);

        for (int i = 0; i < dimension-1; ++i) {
            int current_id = tour.back();
            int next_id = -1;
            double next_cost = -1.0;
            for (int j = 0; j < dimension; ++j) {
                if (std::find(tour.begin(), tour.end(), j) == tour.end()) {
                    if (next_cost < 0 || cost_mat(current_id, j) < next_cost) {
                        next_cost = cost_mat(current_id, j);
                        next_id = j;
                    }
                }
            }
            tour.push_back(next_id);
        }

        // remove the first entry from the tour, as it just refers to the robot's own position
        tour.erase(tour.begin());

        // Decrement each entry by 1 to account for the removed first entry
        for (int& x : tour) {
            x -= 1;
        }
    } else {
        // Solve Asymmetric Traveling Salesman Problem
        // Problem file in TSPLIB format
        {
            std::ofstream prob_file(data_dir_ + "/single.tsp");
            prob_file << "NAME : single\nTYPE : ATSP\nDIMENSION : " << dimension
                    << "\nEDGE_WEIGHT_TYPE : EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\n"
                        "EDGE_WEIGHT_SECTION\n";

            // LKH needs integer costs
            const int scale = 100;
            for (int i = 0; i < dimension; ++i) {
                for (int j = 0; j < dimension; ++j) {
                    int int_cost = static_cast<int>(cost_mat(i, j) * scale);
                    prob_file << int_cost << " ";
                }
                prob_file << "\n";
            }
            prob_file << "EOF";
        }

        RCLCPP_INFO(this->get_logger(), "Starting ATSP solution with LKH...");
        double t1 = this->now().seconds();

        solveTSPLKH((data_dir_ + "/single.par").c_str());

        double t2 = this->now().seconds();
        if (t2-t1 > tsp_max_time_) tsp_max_time_ = t2-t1;
        RCLCPP_INFO(this->get_logger(), "ATSP solution completed in %f seconds (max time: %f seconds)",
                    t2 - t1, tsp_max_time_);

        std::ifstream res_file(data_dir_ + "/single.txt");
        if (!res_file) {
            std::cerr << "Could not open result file " << data_dir_ << "/single.txt" << std::endl;
            return;
        }

        std::string res;
        while (std::getline(res_file, res)) {
            if (res.compare("TOUR_SECTION") == 0) break;
        }

        while (std::getline(res_file, res)) {
            int id = std::stoi(res);
            if (id == 1)  // Ignore the current state
            continue;
            if (id == -1) break;
            tour.push_back(id - 2);  // Idx of solver-2 == Idx of frontier
        }
    }
}

void PathPlanningManager::refineLocalTour(const Eigen::Vector2d& cur_pos,
                                          const Eigen::Vector2d& cur_vel,
                                          const double& cur_yaw,
                                          const std::vector<std::vector<Eigen::Vector2d>>& n_points,
                                          const std::vector<std::vector<double>>& n_yaws,
                                          std::vector<Eigen::Vector2d>& refined_pts,
                                          std::vector<double>& refined_yaws) {
    // Create graph for viewpoints selection
    GraphSearch<ViewNode> g_search;
    std::vector<ViewNode::Ptr> last_group, cur_group;

    // Add the current robot state as the root node
    ViewNode::Ptr first(new ViewNode(cur_pos, cur_yaw));
    first->vel_ = cur_vel;
    g_search.addNode(first);
    last_group.push_back(first);

    // Add provided viewpoints of each cluster to the graph
    // Each viewpoint in the first cluster is a child of the current robot node.
    // Each viewpoint in the second cluster is a child of all viewpoints in the
    // first cluster, and so on.
    for (int i = 0; i < n_points.size(); ++i) {
        // Create nodes for viewpoints of one frontier cluster
        for (int j = 0; j < n_points[i].size(); ++j) {
            ViewNode::Ptr node(new ViewNode(n_points[i][j], n_yaws[i][j]));
            g_search.addNode(node);
            // Connect a node to all nodes in the last group
            for (auto nd : last_group)
                g_search.addEdge(nd->id_, node->id_);
            cur_group.push_back(node);
        }
        // Store nodes for this group for connecting edges
        last_group = cur_group;
        cur_group.clear();
    }

    // Search optimal sequence in the graph
    // The optimal sequence is the one that reaches a leaf node (node in the last
    // cluster) and has the lowest cost.
    std::vector<ViewNode::Ptr> path;
    double t1 = this->now().seconds();
    g_search.FullDijkstraSearch(first->id_, path);
    double t2 = this->now().seconds();
    if (t2-t1 > dijkstra_max_time_) dijkstra_max_time_ = t2-t1;
    RCLCPP_INFO(this->get_logger(), "Dijkstra search completed in %f seconds (max time: %f seconds)",
                t2 - t1, dijkstra_max_time_);
    
    // Return searched sequence
    for (int i = 1; i < path.size(); ++i) {
        refined_pts.push_back(path[i]->pos_);
        refined_yaws.push_back(path[i]->yaw_);
    }
}

void PathPlanningManager::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr odometry_msg) {
    // Get position and yaw
    hippo_position_ << odometry_msg->pose.pose.position.x, odometry_msg->pose.pose.position.y;
    hippo_yaw_ = tf2::getYaw(odometry_msg->pose.pose.orientation);

    // Get velocity in the child frame and transform it to the map frame
    geometry_msgs::msg::Vector3Stamped velocity_child;
    velocity_child.header.stamp = odometry_msg->header.stamp;
    velocity_child.header.frame_id = odometry_msg->child_frame_id;
    velocity_child.vector = odometry_msg->twist.twist.linear;

    geometry_msgs::msg::TransformStamped transform;
    
    // get the transform from camera frame to map frame
    try
    {
      transform = tf_buffer_->lookupTransform(
        "map",                                       // target frame
        odometry_msg->child_frame_id,                // source frame
        odometry_msg->header.stamp,                  // time stamp for the transform
        rclcpp::Duration::from_seconds(0.2));        // timeout for the transform
    }
    catch (const tf2::TransformException & ex)
    {
      RCLCPP_WARN(this->get_logger(), "Could not transform velocity from '%s' to '%s': %s",
                  odometry_msg->child_frame_id.c_str(), "map", ex.what());
      return;
    }
    
    geometry_msgs::msg::Vector3Stamped velocity_map;
    tf2::doTransform(velocity_child, velocity_map, transform);

    hippo_velocity_ << velocity_map.vector.x, velocity_map.vector.y;

    if (!odometry_received_) {
        RCLCPP_INFO(this->get_logger(), "Odometry received by path planner for the first time.");
        odometry_received_ = true;
    }

    visualization_msgs::msg::Marker marker_velocity;
    marker_velocity.header.frame_id = "map";
    marker_velocity.header.stamp = this->now();
    vis_utils_->drawVelocity(marker_velocity, hippo_position_, hippo_velocity_);
    publisher_velocity_->publish(marker_velocity);
}

void PathPlanningManager::publishFrontierStatus(const std::list<FrontierCluster> & frontier_clusters,
                                                bool empty) {
    // Draw frontiers and viewpoints in RViz
    visualization_msgs::msg::Marker marker_cells;
    visualization_msgs::msg::Marker marker_viewpoints;
    visualization_msgs::msg::Marker marker_cluster_centroids;
    visualization_msgs::msg::Marker marker_fov;
    std::list<std::string> frontier_status;

    marker_cells.header.frame_id = "map";
    marker_cells.header.stamp = this->now();
    marker_viewpoints.header = marker_cells.header;
    marker_cluster_centroids.header = marker_cells.header;
    marker_fov.header = marker_cells.header;

    if (empty) {
        marker_cells.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_viewpoints.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_cluster_centroids.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_fov.action = visualization_msgs::msg::Marker::DELETEALL;
    } else {
        // Populate the markers
        vis_utils_->drawFrontiers(marker_cells,
                              marker_viewpoints,
                              marker_cluster_centroids,
                              marker_fov, 
                              frontier_status,
                              frontier_colors_,
                              frontier_clusters,
                              *most_recent_map_);

        // Publish info about the frontiers
        for (const auto & status : frontier_status) {
            auto message = std_msgs::msg::String();
            message.data = status;
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            publisher_string_->publish(message);
        }
    }

    publisher_frontiers_->publish(marker_cells);
    publisher_viewpoints_->publish(marker_viewpoints);
    publisher_cluster_centroids_->publish(marker_cluster_centroids);
    publisher_fov_->publish(marker_fov);
}

void PathPlanningManager::publishPathplanningStatus(const std::vector<Eigen::Vector2d>& global_path,
                                                    const Eigen::Vector2d& next_pos,
                                                    const double next_yaw,
                                                    const std::vector<Eigen::Vector2d>& refined_path,
                                                    bool empty) {
    // Draw the global path, the robot at the next viewpoint, and the local path in RViz
    visualization_msgs::msg::Marker marker_global_path;
    visualization_msgs::msg::Marker marker_next_viewpoint;
    visualization_msgs::msg::Marker marker_refined_views;

    marker_global_path.header.frame_id = "map";
    marker_global_path.header.stamp = this->now();
    marker_next_viewpoint.header = marker_global_path.header;
    marker_refined_views.header = marker_global_path.header;

    if (empty) {
        marker_global_path.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_next_viewpoint.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_refined_views.action = visualization_msgs::msg::Marker::DELETEALL;
    } else {
        vis_utils_->drawPath(marker_global_path, global_path);
        vis_utils_->drawNextViewpoint(marker_next_viewpoint, next_pos, next_yaw);
        vis_utils_->drawPath(marker_refined_views, refined_path);
    }

    publisher_global_path_->publish(marker_global_path);
    publisher_next_viewpoint_->publish(marker_next_viewpoint);
    publisher_refined_views_->publish(marker_refined_views);
}

void PathPlanningManager::publishEmptyMarkers() {
    std::list<FrontierCluster> frontier_clusters;
    std::vector<Eigen::Vector2d> global_path;
    Eigen::Vector2d next_pos;
    double next_yaw;
    std::vector<Eigen::Vector2d> refined_path;
    bool empty = true;

    publishFrontierStatus(frontier_clusters, empty);
    publishPathplanningStatus(global_path, next_pos, next_yaw, refined_path, empty);
}

double PathPlanningManager::wrapYaw(double yaw) {
    while (yaw > 3*M_PI/2) yaw -= 2 * M_PI;
    while (yaw < -M_PI/2) yaw += 2 * M_PI;
    return yaw;
}

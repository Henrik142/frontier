#include "path_planning_manager.hpp"

PathPlanningManager::PathPlanningManager(const std::shared_ptr<Map> & map) : Node("path_planning_manager") {
    // Initialize the map
    map_ = map;
    map_builder_.reset(new MapBuilder(map_.get()));

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
    frontier_finder_->initFrontierFinder(map_, fov_params_, frontier_params_);

    trajectory_planner_ = std::make_shared<TrajectoryPlanner>();
    trajectory_planner_->initTrajectoryPlanner(map_, trajectory_astar_params_);

    ViewNode::astar_.reset(new Astar);
    ViewNode::astar_->init(path_astar_params_, map_);
    ViewNode::map_ = map_;

    double resolution = map_->getResolution();
    Eigen::Vector2d origin(map_->getOriginX(), map_->getOriginY());
    ViewNode::caster_.reset(new RayCaster);
    ViewNode::caster_->setParams(resolution, origin);

    // TF buffer + listener: subscribes to /tf and /tf_static in the background
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Initialize publishers and subscribers
    publisher_trajectory_ = this->create_publisher<nav_msgs::msg::Path>("frontier/trajectory", 10);
    publisher_global_path_ = this->create_publisher<visualization_msgs::msg::Marker>("frontier/global_path", 10);
    publisher_path_to_next_goal_ = this->create_publisher<visualization_msgs::msg::Marker>("frontier/path_to_next_goal", 10);
    publisher_refined_views_ = this->create_publisher<visualization_msgs::msg::Marker>("frontier/refined_views", 10);
    publisher_next_viewpoint_ = this->create_publisher<visualization_msgs::msg::Marker>("frontier/next_viewpoint", 10);
    publisher_frontiers_ = this->create_publisher<visualization_msgs::msg::Marker>("frontier/cells", 10);
    publisher_viewpoints_ = this->create_publisher<visualization_msgs::msg::Marker>("frontier/viewpoints", 10);
    publisher_top_robot_positions_ = this->create_publisher<visualization_msgs::msg::Marker>("frontier/top_robot_positions", 10);
    publisher_fov_ = this->create_publisher<visualization_msgs::msg::Marker>("frontier/viewpoints_fov", 10);
    publisher_velocity_ = this->create_publisher<visualization_msgs::msg::Marker>("velocity", 10);
    publisher_string_ = this->create_publisher<std_msgs::msg::String>("frontier/status", 10);
    publisher_yaw_ = this->create_publisher<std_msgs::msg::Float64>("frontier/next_yaw", 10);

    subscription_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry",
        rclcpp::SensorDataQoS(),   // sensor data QoS: best-effort, suited for high-rate streams
        std::bind(&PathPlanningManager::odometryCallback, this, std::placeholders::_1));

    // Create timer for the path planning initialization
    path_planning_init_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(update_rate_ms_),
        std::bind(&PathPlanningManager::pathPlanningInit, this));
    
    // Create timer that plans a tour over the frontier clusters at a fixed frequency
    path_planning_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(update_rate_ms_),
        std::bind(&PathPlanningManager::pathPlanningCallback, this));

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
    frontier_params_.min_candidate_dist = get_parameter("frontier.min_candidate_dist").as_double();
    RCLCPP_INFO(get_logger(), "frontier.min_candidate_dist=%f", frontier_params_.min_candidate_dist);
    frontier_params_.min_candidate_yaw_diff = get_parameter("frontier.min_candidate_yaw_diff").as_double();
    RCLCPP_INFO(get_logger(), "frontier.min_candidate_yaw_diff=%f", frontier_params_.min_candidate_yaw_diff);
    frontier_params_.min_visib_num = get_parameter("frontier.min_visib_num").as_int();
    RCLCPP_INFO(get_logger(), "frontier.min_visib_num=%d", frontier_params_.min_visib_num);

    // Convert degrees to radians
    frontier_params_.candidate_dphi = frontier_params_.candidate_dphi * M_PI / 180.0;
    frontier_params_.min_candidate_yaw_diff = frontier_params_.min_candidate_yaw_diff * M_PI / 180.0;

    // Path planning parameters
    declare_parameter<int>("path_planning.update_rate_ms", 200);
    declare_parameter<double>("path_planning.shorten_path_dist_thresh", 3.0);
    declare_parameter<bool>("path_planning.refine_local", true);
    declare_parameter<int>("path_planning.refine_num", 2);
    declare_parameter<double>("path_planning.refine_radius", 1.0);
    declare_parameter<double>("path_planning.vm", 1.0);
    declare_parameter<double>("path_planning.yd", 0.7);
    declare_parameter<double>("path_planning.w_y", 1.0);
    declare_parameter<double>("path_planning.w_dir", 1.0);
    declare_parameter<double>("path_planning.astar.resolution", 0.3);
    declare_parameter<double>("path_planning.astar.lambda_heu", 10.0);
    declare_parameter<double>("path_planning.astar.max_search_time", 0.1);
    declare_parameter<int>("path_planning.astar.allocate_num", 100000);

    update_rate_ms_ = get_parameter("path_planning.update_rate_ms").as_int();
    RCLCPP_INFO(get_logger(), "path_planning.update_rate_ms=%d", update_rate_ms_);
    shorten_path_dist_thresh_ = get_parameter("path_planning.shorten_path_dist_thresh").as_double();
    RCLCPP_INFO(get_logger(), "path_planning.shorten_path_dist_thresh=%f", shorten_path_dist_thresh_);
    refine_local_ = get_parameter("path_planning.refine_local").as_bool();
    RCLCPP_INFO(get_logger(), "path_planning.refine_local=%s", refine_local_ ? "true" : "false");
    refine_num_ = get_parameter("path_planning.refine_num").as_int();
    RCLCPP_INFO(get_logger(), "path_planning.refine_num=%d", refine_num_);
    refine_radius_ = get_parameter("path_planning.refine_radius").as_double();
    RCLCPP_INFO(get_logger(), "path_planning.refine_radius=%f", refine_radius_);
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
    path_astar_params_.max_search_time = get_parameter("path_planning.astar.max_search_time").as_double();
    RCLCPP_INFO(get_logger(), "path_planning.astar.max_search_time=%f", path_astar_params_.max_search_time);
    path_astar_params_.allocate_num = get_parameter("path_planning.astar.allocate_num").as_int();
    RCLCPP_INFO(get_logger(), "path_planning.astar.allocate_num=%d", path_astar_params_.allocate_num);

    // Trajectory planning parameters
    declare_parameter<double>("trajectory_planning.astar.resolution", 0.1);
    declare_parameter<double>("trajectory_planning.astar.lambda_heu", 10.0);
    declare_parameter<double>("trajectory_planning.astar.max_search_time", 0.1);
    declare_parameter<int>("trajectory_planning.astar.allocate_num", 100000);

    trajectory_astar_params_.resolution = get_parameter("trajectory_planning.astar.resolution").as_double();
    RCLCPP_INFO(get_logger(), "trajectory_planning.astar.resolution=%f", trajectory_astar_params_.resolution);
    trajectory_astar_params_.lambda_heu = get_parameter("trajectory_planning.astar.lambda_heu").as_double();
    RCLCPP_INFO(get_logger(), "trajectory_planning.astar.lambda_heu=%f", trajectory_astar_params_.lambda_heu);
    trajectory_astar_params_.max_search_time = get_parameter("trajectory_planning.astar.max_search_time").as_double();
    RCLCPP_INFO(get_logger(), "trajectory_planning.astar.max_search_time=%f", trajectory_astar_params_.max_search_time);
    trajectory_astar_params_.allocate_num = get_parameter("trajectory_planning.astar.allocate_num").as_int();
    RCLCPP_INFO(get_logger(), "trajectory_planning.astar.allocate_num=%d", trajectory_astar_params_.allocate_num);
}

void PathPlanningManager::pathPlanningInit() {
    // Initialization logic for path planning goes here

    // only execute if odometry has been received at least once
    if (!odometry_received_)
        return;

    // "discover" a box around the initial position
    if (!init_box_discovered_) {
        Eigen::Vector2d box_mid_pos;
        box_mid_pos.x() = hippo_position_.x();
        box_mid_pos.y() = hippo_position_.y();

        double side_length = 0.5;
        Eigen::Vector2d box_min_pos = box_mid_pos - 0.5 * Eigen::Vector2d::Ones() * side_length;
        Eigen::Vector2d box_max_pos = box_mid_pos + 0.5 * Eigen::Vector2d::Ones() * side_length;
        map_builder_->setBox(box_min_pos, box_max_pos, Map::FREE);
        map_->inflateObstacles();

        init_box_discovered_ = true;
    }

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
            return;
        }

        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = this->now();

        geometry_msgs::msg::PoseStamped pose_hippo;
        pose_hippo.pose.position.x = hippo_position_.x();
        pose_hippo.pose.position.y = hippo_position_.y();
        pose_hippo.pose.position.z = 0.0;

        path_msg.poses.push_back(pose_hippo);
        path_msg.poses.push_back(pose_hippo);

        std_msgs::msg::Float64 target_yaw_msg;
        target_yaw_msg.data = target_yaw;

        publisher_trajectory_->publish(path_msg);
        publisher_yaw_->publish(target_yaw_msg);
    }
}

int PathPlanningManager::pathPlanningCallback() {
    // only execute if path planning has been initialized
    if (!path_planning_initialized_)
        return WAITING_FOR_INIT;

    // find frontiers, sample viewpoints
    frontier_finder_->findFrontiers();
    frontier_finder_->computeClustersToVisit();
    std::list<FrontierCluster> frontier_clusters = frontier_finder_->getFrontierClusters();

    // Publish the frontiers
    publishFrontierStatus(frontier_clusters);

    if (frontier_clusters.empty()) {
        RCLCPP_INFO(this->get_logger(), "No frontier clusters available.");
        return NO_FRONTIER;
    }

    // Get cost matrix for current state and clusters
    Eigen::MatrixXd cost_mat;
    RCLCPP_INFO(this->get_logger(), "Updating frontier cost matrix");
    frontier_finder_->updateFrontierCostMatrix();
    frontier_finder_->getFullCostMatrix(hippo_position_, hippo_velocity_, hippo_yaw_, cost_mat);

    // Print the cost matrix
    std::ostringstream oss;
    for (int i = 0; i < cost_mat.cols(); ++i) {
        if (i > 0) oss << ",";
        oss << cost_mat(0, i);
    }
    std::string result = oss.str();
    RCLCPP_INFO(this->get_logger(), "Costs from hippo_position_: [%s]", result.c_str());

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
    std::vector<Eigen::Vector2d> path;
    frontier_finder_->getPathForTour(hippo_position_, tour, path);
    RCLCPP_INFO(this->get_logger(), "Global path has %zu points.", path.size());

    // Publish the global path
    visualization_msgs::msg::Marker marker_global_path;
    marker_global_path.header.frame_id = "map";
    marker_global_path.header.stamp = this->now();
    vis_utils_->drawPath(marker_global_path, path);
    publisher_global_path_->publish(marker_global_path);

    // Find the target position and yaw for trajectory planning
    Eigen::Vector2d next_pos;
    double next_yaw;

    // Refine the local tour if the flag is enabled
    if (refine_local_) {
        std::vector<Eigen::Vector2d> unrefined_pts;
        std::vector<int> refined_ids;

        // Number of clusters to consider in the refinement
        int k_num = std::min(refine_num_, static_cast<int>(tour.size()));

        // Make a cluster_indexer to access the frontier list easier
        std::vector<std::list<FrontierCluster>::iterator> cluster_indexer;
        for (auto it = frontier_clusters.begin(); it != frontier_clusters.end(); ++it)
            cluster_indexer.push_back(it);

        // Get the top viewpoint positions of the first k clusters in the tour
        for (int i = 0; i < k_num; ++i) {
            auto tmp = cluster_indexer[tour[i]]->viewpoints_.front().robot_pos_;
            unrefined_pts.push_back(tmp);
            refined_ids.push_back(tour[i]);
            if ((tmp - hippo_position_).norm() > refine_radius_ && refined_ids.size() >= 2) break;
        }

        // Get top N viewpoints for the next K frontiers
        std::vector<std::vector<Eigen::Vector2d>> n_points;
        std::vector<std::vector<double>> n_yaws;
        int N = 6;
        double max_decay = 0.5;
        frontier_finder_->getViewpointsInfo(hippo_position_, hippo_yaw_, refined_ids, N, max_decay, n_points, n_yaws);

        // Calculate the refined positions and yaws
        std::vector<Eigen::Vector2d> refined_points;
        std::vector<double> refined_yaws;
        refineLocalTour(hippo_position_, hippo_velocity_, hippo_yaw_, n_points, n_yaws, refined_points, refined_yaws);

        next_pos = refined_points.front();
        next_yaw = refined_yaws.front();

        // Publish the refined viewpoints
        visualization_msgs::msg::Marker marker_refined_views;
        marker_refined_views.header.frame_id = "map";
        marker_refined_views.header.stamp = this->now();
        vis_utils_->drawPath(marker_refined_views, refined_points);
        publisher_refined_views_->publish(marker_refined_views);
    } else {
        // Make a cluster_indexer to access the frontier list easier
        std::vector<std::list<FrontierCluster>::iterator> cluster_indexer;
        for (auto it = frontier_clusters.begin(); it != frontier_clusters.end(); ++it)
            cluster_indexer.push_back(it);
        
        // Simply pick the top viewpoint of the first cluster in the tour
        next_pos = cluster_indexer[tour.front()]->viewpoints_.front().robot_pos_;
        next_yaw = cluster_indexer[tour.front()]->viewpoints_.front().yaw_;
    }

    // Plan trajectory to next viewpoint with increasing resolution
    bool trajectory_found = false;
    auto resolutions = {0.3, 0.2, 0.1};
    for (auto res : resolutions) {
        trajectory_planner_->astar_->reset();
        trajectory_planner_->astar_->setResolution(res);
        if (trajectory_planner_->astar_->search(hippo_position_, next_pos) == Astar::REACH_END) {
            trajectory_found = true;
            break;
        }
    }

    /*
    // Retrieve the next viewpoint to visit and find a feasible trajectory to it
    // First, go through all viewpoints of the first cluster in the tour (from best to worst)
    // and try to find a path. As soon as a feasible trajectory is found, we stop searching further.
    // If no feasible trajectory is found for any viewpoint in the first cluster, we move on to
    // the next cluster in the tour. This is repeated until a feasible trajectory is found or all
    // clusters have been exhausted.
    for (int next_cluster_id : tour) {
        for (auto & frontier_cluster : frontier_clusters) {
            if (frontier_cluster.id_ != next_cluster_id) {
                continue;
            }

            for (std::size_t i = 0; i < frontier_cluster.viewpoints_.size(); ++i) {
                auto & viewpoint = frontier_cluster.viewpoints_[i];
                next_pos = viewpoint.robot_pos_;
                next_yaw = viewpoint.yaw_;

                // When considering the best viewpoint:
                // Skip to the second-best viewpoint if the Hippo is already
                // sufficiently close to and aligned with the best one
                if (i == 0 &&
                    (hippo_position_ - next_pos).norm() < 0.05 && 
                    std::abs(hippo_yaw_ - next_yaw) < 0.1) {
                    continue;
                }

                // Plan trajectory to next viewpoint with increasing resolution
                auto resolutions = {0.3, 0.2, 0.1};
                for (auto res : resolutions) {
                    trajectory_planner_->astar_->reset();
                    trajectory_planner_->astar_->setResolution(res);
                    if (trajectory_planner_->astar_->search(hippo_position_, next_pos) == Astar::REACH_END) {
                        trajectory_found = true;
                        break;
                    }
                }

                if (trajectory_found) {
                    RCLCPP_INFO(this->get_logger(), "Next viewpoint position: [%f, %f], yaw: [%f]", next_pos.x(), next_pos.y(), next_yaw);
                    break;
                }
            }
            
            break;
            
        }
        if (trajectory_found) {
            break;
        }
    }
    */

    if (!trajectory_found) {
        RCLCPP_WARN(this->get_logger(), "Failed to find path to any viewpoint.");
        return FAIL;
    }

    std::vector<Eigen::Vector2d> path_next_goal = trajectory_planner_->astar_->getPath();
    shortenPath(path_next_goal);

    // TODO: Improve the trajectory planning, e.g., by smoothing the path,
    // ensuring dynamic feasibility, and avoiding obstacles.

    /*
    Publish everything
    */

    // Publish the target yaw of the next viewpoint
    std_msgs::msg::Float64 yaw_msg;
    yaw_msg.data = next_yaw;
    publisher_yaw_->publish(yaw_msg);

    // Publish the next viewpoint for rviz
    visualization_msgs::msg::Marker marker_next_viewpoint;
    marker_next_viewpoint.header.frame_id = "map";
    marker_next_viewpoint.header.stamp = this->now();
    vis_utils_->drawNextViewpoint(marker_next_viewpoint, next_pos, next_yaw);
    publisher_next_viewpoint_->publish(marker_next_viewpoint);

    // Publish the path to the next goal
    // This is what the path follower subscribes to
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = this->now();
    for (const auto & point : path_next_goal) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = point.x();
        pose.pose.position.y = point.y();
        pose.pose.position.z = 0.0;
        path_msg.poses.push_back(pose);
    }
    publisher_trajectory_->publish(path_msg);

    // Publish path markers for visualization
    visualization_msgs::msg::Marker marker_path_to_next_goal;
    marker_path_to_next_goal.header.frame_id = "map";
    marker_path_to_next_goal.header.stamp = this->now();
    vis_utils_->drawPath(marker_path_to_next_goal, path_next_goal);
    publisher_path_to_next_goal_->publish(marker_path_to_next_goal);

    RCLCPP_INFO(this->get_logger(), "Path to next viewpoint published.");
    return SUCCEED;
}

void PathPlanningManager::findGlobalTour(std::vector<int> & tour,
                                    const Eigen::MatrixXd & cost_mat) {
    // For now: find "greedy" global tour that passes all clusters
    // by first going to the cluster with the lowest cost, then going to the
    // next cluster that has the lowest cost from the first one, and so on.
    // TODO: Implement solution of Traveling Salesman Problem
    tour.clear();
    tour.push_back(0);
    int num_clusters = cost_mat.cols();

    for (int i = 0; i < num_clusters-1; ++i) {
        int current_id = tour.back();
        int next_id = -1;
        double next_cost = -1.0;
        for (int j = 0; j < num_clusters; ++j) {
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

    // Add the current state
    ViewNode::Ptr first(new ViewNode(cur_pos, cur_yaw));
    first->vel_ = cur_vel;
    g_search.addNode(first);
    last_group.push_back(first);
    ViewNode::Ptr final_node;

    // Add viewpoints
    for (int i = 0; i < n_points.size(); ++i) {
        // Create nodes for viewpoints of one frontier
        for (int j = 0; j < n_points[i].size(); ++j) {
            ViewNode::Ptr node(new ViewNode(n_points[i][j], n_yaws[i][j]));
            g_search.addNode(node);
            // Connect a node to nodes in last group
            for (auto nd : last_group)
                g_search.addEdge(nd->id_, node->id_);
            cur_group.push_back(node);

            // Only keep the first viewpoint of the last local frontier
            if (i == n_points.size() - 1) {
                final_node = node;
                break;
            }
        }
        // Store nodes for this group for connecting edges
        last_group = cur_group;
        cur_group.clear();
    }

    // Search optimal sequence
    std::vector<ViewNode::Ptr> path;
    g_search.DijkstraSearch(first->id_, final_node->id_, path);

    // Return searched sequence
    for (int i = 1; i < path.size(); ++i) {
        refined_pts.push_back(path[i]->pos_);
        refined_yaws.push_back(path[i]->yaw_);
    }
}

void PathPlanningManager::shortenPath(std::vector<Eigen::Vector2d>& path) {
  if (path.empty()) {
    return;
  }

  // Shorten the tour, only critical intermediate points are reserved.
  std::vector<Eigen::Vector2d> short_tour = { path.front() };
  for (int i = 1; i < path.size() - 1; ++i) {
    if ((path[i] - short_tour.back()).norm() > shorten_path_dist_thresh_)
      short_tour.push_back(path[i]);
    else {
      // Add waypoints to shorten path only to avoid collision
      ViewNode::caster_->input(short_tour.back(), path[i + 1]);
      Eigen::Vector2i idx;
      while (ViewNode::caster_->nextId(idx)) {
        if (map_->getInflatedOccupancy(idx) != Map::FREE) {
          short_tour.push_back(path[i]);
          break;
        }
      }
    }
  }
  if ((path.back() - short_tour.back()).norm() > 1e-3) short_tour.push_back(path.back());

  // Ensure at least three points in the path
  if (short_tour.size() == 2)
    short_tour.insert(short_tour.begin() + 1, 0.5 * (short_tour[0] + short_tour[1]));
  path = short_tour;
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

void PathPlanningManager::publishFrontierStatus(const std::list<FrontierCluster> & frontier_clusters) {
    // Draw frontiers and viewpoints in RViz
    visualization_msgs::msg::Marker marker_cells;
    visualization_msgs::msg::Marker marker_viewpoints;
    visualization_msgs::msg::Marker marker_top_robot_positions;
    visualization_msgs::msg::Marker marker_fov;
    std::list<std::string> frontier_status;

    marker_cells.header.frame_id = "map";
    marker_cells.header.stamp = this->now();
    marker_viewpoints.header = marker_cells.header;
    marker_top_robot_positions.header = marker_cells.header;
    marker_fov.header = marker_cells.header;

    if (frontier_clusters.empty()) {
        marker_cells.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_viewpoints.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_top_robot_positions.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_fov.action = visualization_msgs::msg::Marker::DELETEALL;
        publisher_frontiers_->publish(marker_cells);
        publisher_viewpoints_->publish(marker_viewpoints);
        publisher_top_robot_positions_->publish(marker_top_robot_positions);
        publisher_fov_->publish(marker_fov);
        return;
    }

    vis_utils_->drawFrontiers(marker_cells,
                              marker_viewpoints,
                              marker_top_robot_positions,
                              marker_fov, 
                              frontier_status,
                              frontier_colors_,
                              frontier_clusters,
                              *map_);

    publisher_frontiers_->publish(marker_cells);
    publisher_viewpoints_->publish(marker_viewpoints);
    publisher_top_robot_positions_->publish(marker_top_robot_positions);
    publisher_fov_->publish(marker_fov);
    
    // Publish info about the frontiers
    for (const auto & status : frontier_status) {
        auto message = std_msgs::msg::String();
        message.data = status;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_string_->publish(message);
    }
}

double PathPlanningManager::wrapYaw(double yaw) {
    while (yaw > 3*M_PI/2) yaw -= 2 * M_PI;
    while (yaw < -M_PI/2) yaw += 2 * M_PI;
    return yaw;
}

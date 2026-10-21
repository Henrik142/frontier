#include "trajectory_planner.hpp"

TrajectoryPlanner::TrajectoryPlanner(
      const std::shared_ptr<PlanningMaps> & planning_maps)
      : Node("trajectory_planner") {
  // Initialize the map
  planning_maps_ = planning_maps;
  most_recent_map_ = planning_maps_->acquire();

  initParams();

  vis_utils_.reset(new VisualizationUtils());
  vis_utils_->initVisUtils(fov_params_);

  astar_.reset(new Astar);
  astar_->init(astar_params_, most_recent_map_);

  // Get map params
  double resolution = most_recent_map_->getResolution();
  Eigen::Vector2d origin;
  origin << most_recent_map_->getOriginX(), most_recent_map_->getOriginY();

  raycaster_.reset(new RayCaster);
  raycaster_->setParams(resolution, origin);

  publisher_trajectory_ = this->create_publisher<nav_msgs::msg::Path>("frontier/trajectory", 10);
  publisher_path_to_next_goal_ = this->create_publisher<visualization_msgs::msg::Marker>("frontier/path_to_next_goal", 10);

  subscription_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odometry",
    rclcpp::SensorDataQoS(),
    std::bind(&TrajectoryPlanner::odometryCallback, this, std::placeholders::_1));

  subscription_pos_target_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
    "frontier/next_pos",
    rclcpp::SensorDataQoS(),
    std::bind(&TrajectoryPlanner::posTargetCallback, this, std::placeholders::_1));

  // Create timer that generates the trajectory at a fixed frequency
  trajectory_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(update_rate_ms_),
    std::bind(&TrajectoryPlanner::planTrajectory, this));
}

TrajectoryPlanner::~TrajectoryPlanner() {
}

void TrajectoryPlanner::initParams() {
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

  // Trajectory planning parameters
  declare_parameter<int>("trajectory_planning.update_rate_ms", 100);
  declare_parameter<double>("path_planning.shorten_path_dist_thresh", 3.0);
  declare_parameter<double>("trajectory_planning.astar.resolution", 0.1);
  declare_parameter<double>("trajectory_planning.astar.lambda_heu", 10.0);
  declare_parameter<double>("trajectory_planning.astar.max_search_time", 0.1);
  declare_parameter<int>("trajectory_planning.astar.allocate_num", 100000);

  update_rate_ms_ = get_parameter("trajectory_planning.update_rate_ms").as_int();
  RCLCPP_INFO(get_logger(), "trajectory_planning.update_rate_ms=%d", update_rate_ms_);
  shorten_path_dist_thresh_ = get_parameter("path_planning.shorten_path_dist_thresh").as_double();
  RCLCPP_INFO(get_logger(), "path_planning.shorten_path_dist_thresh=%f", shorten_path_dist_thresh_);
  astar_params_.resolution = get_parameter("trajectory_planning.astar.resolution").as_double();
  RCLCPP_INFO(get_logger(), "trajectory_planning.astar.resolution=%f", astar_params_.resolution);
  astar_params_.lambda_heu = get_parameter("trajectory_planning.astar.lambda_heu").as_double();
  RCLCPP_INFO(get_logger(), "trajectory_planning.astar.lambda_heu=%f", astar_params_.lambda_heu);
  astar_params_.max_search_time = get_parameter("trajectory_planning.astar.max_search_time").as_double();
  RCLCPP_INFO(get_logger(), "trajectory_planning.astar.max_search_time=%f", astar_params_.max_search_time);
  astar_params_.allocate_num = get_parameter("trajectory_planning.astar.allocate_num").as_int();
  RCLCPP_INFO(get_logger(), "trajectory_planning.astar.allocate_num=%d", astar_params_.allocate_num);
}

void TrajectoryPlanner::planTrajectory() {
  if (!odometry_received_ || !pos_target_received_) {
    return;
  }

  double t1 = this->now().seconds();

  std::vector<Eigen::Vector2d> path_next_goal;

  if ((next_pos_ - hippo_position_).norm() < 1e-3) {
    path_next_goal.push_back(hippo_position_);
    path_next_goal.push_back(next_pos_);
  } else {
    most_recent_map_ = planning_maps_->acquire();
    astar_->setMap(most_recent_map_);

    // Plan trajectory to next viewpoint with increasing resolution
    bool trajectory_found = false;
    auto resolutions = {0.3, 0.2, 0.1};
    for (auto res : resolutions) {
      astar_->reset();
      astar_->setResolution(res);
      if (astar_->search(hippo_position_, next_pos_) == Astar::REACH_END) {
        trajectory_found = true;
        break;
      }
    }

    if (!trajectory_found) {
      RCLCPP_WARN(this->get_logger(), "Failed to find path to target viewpoint.");
      return;
    }

    path_next_goal = astar_->getPath();
    shortenPath(path_next_goal);
  }

  // TODO: Improve the trajectory planning, e.g., by smoothing the path,
  // ensuring dynamic feasibility, and avoiding obstacles.    

  // Publish the path to the next goal
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

  // Publish for Rviz
  visualization_msgs::msg::Marker marker_path_to_next_goal;
  marker_path_to_next_goal.header.frame_id = "map";
  marker_path_to_next_goal.header.stamp = this->now();
  vis_utils_->drawPath(marker_path_to_next_goal, path_next_goal);
  publisher_path_to_next_goal_->publish(marker_path_to_next_goal);

  double t2 = this->now().seconds();
  RCLCPP_INFO(this->get_logger(), "Trajectory planning completed in %f seconds", t2 - t1);
}

void TrajectoryPlanner::shortenPath(std::vector<Eigen::Vector2d>& path) {
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
      raycaster_->input(short_tour.back(), path[i + 1]);
      Eigen::Vector2i idx;
      while (raycaster_->nextId(idx)) {
        if (most_recent_map_->getInflatedOccupancy(idx) != Map::FREE) {
          short_tour.push_back(path[i]);
          break;
        }
      }
    }
  }
  if ((path.back() - short_tour.back()).norm() > 1e-3) short_tour.push_back(path.back());

  // Ensure at least three points in the path
  /*
  if (short_tour.size() == 2)
    short_tour.insert(short_tour.begin() + 1, 0.5 * (short_tour[0] + short_tour[1]));
  */

  path = short_tour;
}

void TrajectoryPlanner::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr odometry_msg) {
  // Get position
  hippo_position_ << odometry_msg->pose.pose.position.x, odometry_msg->pose.pose.position.y;
  odometry_received_ = true;
}

void TrajectoryPlanner::posTargetCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr pos_target_msg) {
  // Handle the received target position, ignore z component
  next_pos_ << pos_target_msg->vector.x, pos_target_msg->vector.y;
  pos_target_received_ = true;
} 
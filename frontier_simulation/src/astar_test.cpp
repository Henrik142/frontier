#include "astar_test.hpp"

AstarTest::AstarTest() : Node("astar_test") {
    initParams();

    map_.reset(new Map());
    map_->initMap(map_params_);

    vis_utils_.reset(new VisualizationUtils());
    vis_utils_->initVisUtils(fov_params_);

    astar_path_searcher_.reset(new Astar());
    astar_path_searcher_->init(astar_params_, map_);

    publisher_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "occupancy_grid", rclcpp::QoS(1).transient_local());
    publisher_path_ = this->create_publisher<visualization_msgs::msg::Marker>("path", 10);
    publisher_start_ = this->create_publisher<visualization_msgs::msg::Marker>("start", 10);
    publisher_goal_ = this->create_publisher<visualization_msgs::msg::Marker>("goal", 10);
    publisher_visited_ = this->create_publisher<visualization_msgs::msg::Marker>("visited", 10);

    MapBuilder map_builder(map_.get());
    map_builder.buildTestScene();
    map_->inflateObstacles();

    map_update_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&AstarTest::mapUpdateCallback, this));
}

AstarTest::~AstarTest() {
}

void AstarTest::initParams() {
    // FOV parameters
    declare_parameter<double>("fov.left_angle", -45.0);
    declare_parameter<double>("fov.right_angle", 45.0);
    declare_parameter<double>("fov.max_dist", 0.6);

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

    map_params_.max_ray_length = fov_params_.max_dist;
    RCLCPP_INFO(get_logger(), "map.max_ray_length=%f", map_params_.max_ray_length);

    // Astar parameters
    declare_parameter<double>("path_planning.astar.resolution", 0.4);
    declare_parameter<double>("path_planning.astar.lambda_heu", 10.0);
    declare_parameter<double>("path_planning.astar.max_search_time", 0.1);
    declare_parameter<int>("path_planning.astar.allocate_num", 100000);

    astar_params_.resolution = get_parameter("path_planning.astar.resolution").as_double();
    RCLCPP_INFO(get_logger(), "path_planning.astar.resolution=%f", astar_params_.resolution);
    astar_params_.lambda_heu = get_parameter("path_planning.astar.lambda_heu").as_double();
    RCLCPP_INFO(get_logger(), "path_planning.astar.lambda_heu=%f", astar_params_.lambda_heu);
    astar_params_.max_search_time = get_parameter("path_planning.astar.max_search_time").as_double();
    RCLCPP_INFO(get_logger(), "path_planning.astar.max_search_time=%f", astar_params_.max_search_time);
    astar_params_.allocate_num = get_parameter("path_planning.astar.allocate_num").as_int();
    RCLCPP_INFO(get_logger(), "path_planning.astar.allocate_num=%d", astar_params_.allocate_num);
}

void AstarTest::testPoints() {
    double origin_x = map_->getOriginX();
    double origin_y = map_->getOriginY();
    double size_x = map_->getSizeX();
    double size_y = map_->getSizeY();

    declare_parameter<int>("astar_test.num_attempts", 1000);
    const int num_attempts = get_parameter("astar_test.num_attempts").as_int();
    const int max_sampling_attempts = std::max(10 * num_attempts, num_attempts);
    int success_cnt = 0;
    int fail_timeout_cnt = 0;
    int fail_memory_cnt = 0;
    int fail_invalid_cnt = 0;
    int search_result;
    std::vector<double> durations;
    std::vector<double> path_lengths;

    int attempt = 0;
    int sampling_attempts = 0;
    while (attempt < num_attempts && sampling_attempts < max_sampling_attempts && rclcpp::ok()) {
        ++sampling_attempts;
        //Eigen::Vector2d start(0.0, 0.0);
        //Eigen::Vector2d goal(1.4, 1.1);

        Eigen::Vector2d start(randomDouble()*5.0 + origin_x + 2.5,
                            randomDouble()*5.0 + origin_y + 2.5);
        Eigen::Vector2d goal(randomDouble()*5.0 + origin_x + 2.5,
                            randomDouble()*5.0 + origin_y + 2.5);

        //Eigen::Vector2d start(seededRandomDouble()*5.0 + origin_x + 2.5,
        //                    seededRandomDouble()*5.0 + origin_y + 2.5);
        //Eigen::Vector2d goal(seededRandomDouble()*5.0 + origin_x + 2.5,
        //                    seededRandomDouble()*5.0 + origin_y + 2.5);

        if (!(map_->getInflatedOccupancy(start)==Map::FREE &&
              map_->getInflatedOccupancy(goal)==Map::FREE)) {
            continue;
        }

        search_result = pathUpdate(start, goal, durations, path_lengths);
        if (search_result == Astar::REACH_END) {
            success_cnt++;
        } else if (search_result == Astar::NO_PATH_TIMEOUT) {
            fail_timeout_cnt++;
        } else if (search_result == Astar::NO_PATH_MEMORY) {
            fail_memory_cnt++;
        } else if (search_result == Astar::NO_PATH_INVALID) {
            fail_invalid_cnt++;
        }

        ++attempt;
    }
    if (durations.empty()) {
        RCLCPP_WARN(get_logger(), "No valid test points were sampled.");
        return;
    }

    RCLCPP_INFO(get_logger(), "Average path search duration: %f seconds", 
                std::accumulate(durations.begin(), durations.end(), 0.0) / durations.size());
    RCLCPP_INFO(get_logger(), "Maximum path search duration: %f seconds", 
                *std::max_element(durations.begin(), durations.end()));
    RCLCPP_INFO(get_logger(), "Average path length: %f", 
                std::accumulate(path_lengths.begin(), path_lengths.end(), 0.0) / path_lengths.size());
    if (attempt < num_attempts && rclcpp::ok()) {
        RCLCPP_WARN(get_logger(), "Only completed %d/%d attempts after %d samples.",
                    attempt, num_attempts, sampling_attempts);
    }
    
    RCLCPP_INFO(get_logger(), "Test results:");
    RCLCPP_INFO(get_logger(), "Success: %d", success_cnt);
    RCLCPP_INFO(get_logger(), "Fail (timeout): %d", fail_timeout_cnt);
    RCLCPP_INFO(get_logger(), "Fail (memory): %d", fail_memory_cnt);
    RCLCPP_INFO(get_logger(), "Fail (invalid): %d", fail_invalid_cnt);
}

void AstarTest::mapUpdateCallback() {
    nav_msgs::msg::OccupancyGrid grid_msg;
    grid_msg.header.stamp = this->now();
    grid_msg.header.frame_id = "map";

    vis_utils_->drawMap(grid_msg, *map_);
    publisher_map_->publish(grid_msg);

    if (!test_points_run_) {
        test_points_run_ = true;
        testPoints();
    }
}

int AstarTest::pathUpdate(Eigen::Vector2d start, Eigen::Vector2d goal,
                            std::vector<double> & durations,
                            std::vector<double> & path_lengths) {
    
    astar_path_searcher_->reset();

    const auto t1 = std::chrono::steady_clock::now();
    int search_result = astar_path_searcher_->search(start, goal);
    const auto t2 = std::chrono::steady_clock::now();
    double duration = std::chrono::duration<double>(t2 - t1).count(); 

    if (search_result == Astar::REACH_END) {
        RCLCPP_INFO(get_logger(), "Path found successfully.");
        
        auto path = astar_path_searcher_->getPath();

        durations.push_back(duration);
        path_lengths.push_back(astar_path_searcher_->pathLength(path));

        visualization_msgs::msg::Marker path_marker;
        path_marker.header.stamp = this->now();
        path_marker.header.frame_id = "map";
        vis_utils_->drawPath(path_marker, path);
        publisher_path_->publish(path_marker);

    } else if (search_result == Astar::NO_PATH_TIMEOUT) {
        RCLCPP_WARN(get_logger(), "Path search timed out.");
    } else if (search_result == Astar::NO_PATH_MEMORY) {
        RCLCPP_ERROR(get_logger(), "Path search failed due to memory limit.");
    } else if (search_result == Astar::NO_PATH_INVALID) {
        RCLCPP_ERROR(get_logger(), "Path search failed, no valid path exists.");
    }

    std::vector<Eigen::Vector2d> start_vector;
    start_vector.push_back(start);
    std::vector<double> start_color = {0.0, 1.0, 0.0, 1.0};

    std::vector<Eigen::Vector2d> goal_vector;
    goal_vector.push_back(goal);
    std::vector<double> goal_color = {1.0, 0.0, 0.0, 1.0};

    visualization_msgs::msg::Marker start_marker;
    drawMarker(start_marker, start_vector, 0.1, start_color);
    publisher_start_->publish(start_marker);

    visualization_msgs::msg::Marker goal_marker;
    drawMarker(goal_marker, goal_vector, 0.1, goal_color);
    publisher_goal_->publish(goal_marker);

    std::vector<Eigen::Vector2d> visited;
    visited = astar_path_searcher_->getVisited();
    std::vector<double> visited_color = {0.0, 0.0, 1.0, 1.0};
    RCLCPP_INFO(get_logger(), "Visited points: %zu", visited.size());

    visualization_msgs::msg::Marker visited_marker;
    drawMarker(visited_marker, visited, 0.03, visited_color);
    publisher_visited_->publish(visited_marker);

    return search_result;
}

void AstarTest::drawMarker(visualization_msgs::msg::Marker & msg_marker,
                           const std::vector<Eigen::Vector2d> & points,
                           const double & scale,
                           const std::vector<double> & color) {
    msg_marker.header.stamp = this->now();
    msg_marker.header.frame_id = "map";
    msg_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    msg_marker.action = visualization_msgs::msg::Marker::ADD;
    msg_marker.scale.x = scale;
    msg_marker.scale.y = scale;
    msg_marker.scale.z = scale;
    msg_marker.color.r = color[0];
    msg_marker.color.g = color[1];
    msg_marker.color.b = color[2];
    msg_marker.color.a = color[3];

    for (const auto & point : points) {
        geometry_msgs::msg::Point p;
        p.x = point.x();
        p.y = point.y();
        p.z = 0.0;
        msg_marker.points.push_back(p);
    }
}

double AstarTest::randomDouble() {
    // returns random double between 0 and 1
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<double> dist(0.0, 1.0);
    return dist(gen);
}

double AstarTest::seededRandomDouble() {
    static std::mt19937 gen(12345);  // fixed seed
    static std::uniform_real_distribution<double> dist(0.0, 1.0);
    return dist(gen);
}


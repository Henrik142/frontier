#include "visualization_utils.hpp"

VisualizationUtils::VisualizationUtils() {
}

VisualizationUtils::~VisualizationUtils() {
}

void VisualizationUtils::initVisUtils(const PerceptionUtils::InitFOVParams& fov_params) {
    fov_left_angle_ = fov_params.left_angle;
    fov_right_angle_ = fov_params.right_angle;
    fov_max_dist_ = fov_params.max_dist;
}

void VisualizationUtils::drawRobotPose(visualization_msgs::msg::Marker & marker_pos,
        const Eigen::Vector2d & position, const double yaw) {
    marker_pos.ns = "robot_pose";
    marker_pos.id = 0;
    marker_pos.type = visualization_msgs::msg::Marker::ARROW;
    marker_pos.action = visualization_msgs::msg::Marker::ADD;
    
    marker_pos.pose.position.x = position.x() - 0.2 * std::cos(yaw);
    marker_pos.pose.position.y = position.y() - 0.2 * std::sin(yaw);
    marker_pos.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    marker_pos.pose.orientation.x = q.x();
    marker_pos.pose.orientation.y = q.y();
    marker_pos.pose.orientation.z = q.z();
    marker_pos.pose.orientation.w = q.w();

    marker_pos.scale.x = 0.4;
    marker_pos.scale.y = 0.05;
    marker_pos.scale.z = 0.05;
    marker_pos.color.r = 0.0;
    marker_pos.color.g = 1.0;
    marker_pos.color.b = 0.0;
    marker_pos.color.a = 1.0;
}

void VisualizationUtils::drawRobotPose(visualization_msgs::msg::Marker & marker_pos,
        const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & pose_msg) {
    marker_pos.header = pose_msg->header;
    marker_pos.ns = "robot_pose";
    marker_pos.id = 0;
    marker_pos.type = visualization_msgs::msg::Marker::ARROW;
    marker_pos.action = visualization_msgs::msg::Marker::ADD;
    
    marker_pos.pose.position.x = pose_msg->pose.pose.position.x - 0.2 * std::cos(tf2::getYaw(pose_msg->pose.pose.orientation));
    marker_pos.pose.position.y = pose_msg->pose.pose.position.y - 0.2 * std::sin(tf2::getYaw(pose_msg->pose.pose.orientation));
    marker_pos.pose.position.z = 0.0;
    marker_pos.pose.orientation = pose_msg->pose.pose.orientation;

    marker_pos.scale.x = 0.4;
    marker_pos.scale.y = 0.05;
    marker_pos.scale.z = 0.05;
    marker_pos.color.r = 0.0;
    marker_pos.color.g = 1.0;
    marker_pos.color.b = 0.0;
    marker_pos.color.a = 1.0;
}

void VisualizationUtils::drawRobotFOV(visualization_msgs::msg::Marker & marker_fov,
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr & camera_pose_msg) {
    marker_fov.header = camera_pose_msg->header;
    marker_fov.ns = "fov";
    marker_fov.id = 0;
    marker_fov.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker_fov.action = visualization_msgs::msg::Marker::ADD;
    marker_fov.scale.x = 0.02;  // line width
    marker_fov.color.r = 1.0;
    marker_fov.color.g = 1.0;
    marker_fov.color.b = 0.0;
    marker_fov.color.a = 0.8;
    
    createFOVMarkers(marker_fov, camera_pose_msg->pose.position.x, camera_pose_msg->pose.position.y,
        tf2::getYaw(camera_pose_msg->pose.orientation));
}

void VisualizationUtils::drawRobotFOV(visualization_msgs::msg::Marker & marker_fov,
    const Eigen::Vector2d & camera_pos, const double yaw) {
    marker_fov.ns = "fov";
    marker_fov.id = 0;
    marker_fov.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker_fov.action = visualization_msgs::msg::Marker::ADD;
    marker_fov.scale.x = 0.02;  // line width
    marker_fov.color.r = 1.0;
    marker_fov.color.g = 1.0;
    marker_fov.color.b = 0.0;
    marker_fov.color.a = 0.8;

    createFOVMarkers(marker_fov, camera_pos.x(), camera_pos.y(), yaw);
}

void VisualizationUtils::drawMap(nav_msgs::msg::OccupancyGrid & grid_msg, const Map & map) {
    int CellsTotal = map.getVoxelNum();
    int CellsX = map.getVoxelNumX();
    int CellsY = map.getVoxelNumY();

    grid_msg.info.resolution = map.getResolution();
    grid_msg.info.width = CellsX;
    grid_msg.info.height = CellsY;
    grid_msg.info.origin.position.x = map.getOriginX();
    grid_msg.info.origin.position.y = map.getOriginY();
    grid_msg.info.origin.orientation.w = 1.0;

    grid_msg.data.resize(CellsTotal);
    for (int i = 0; i < CellsX; ++i) {
        for (int j = 0; j < CellsY; ++j) {
            int occ = map.getOccupancy(Eigen::Vector2i(i, j));
            //int occ = map.getInflatedOccupancy(Eigen::Vector2i(i, j));
            int address = map.toAddress(i, j);
            if (occ == Map::UNKNOWN) {
                grid_msg.data[address] = -1;
            } else if (occ == Map::FREE) {
                grid_msg.data[address] = 0;
            } else if (occ == Map::OCCUPIED) {
                grid_msg.data[address] = 100;
            }
        }
    }
}

void VisualizationUtils::drawFrontiers(visualization_msgs::msg::Marker & marker_cells,
        visualization_msgs::msg::Marker & marker_viewpoints,
        visualization_msgs::msg::Marker & marker_robot_pos,
        visualization_msgs::msg::Marker & marker_fov,
        std::list<std::string> & frontier_status,
        std::vector<std::array<double, 3>> & frontier_colors,
        const std::list<FrontierCluster> & frontier_clusters,
        const Map & map) {
    frontier_status.clear();
    frontier_status.push_back("Updating frontiers");
    
    // Markers for frontier cells
    marker_cells.ns = "frontiers";
    marker_cells.id = 0;
    marker_cells.type = visualization_msgs::msg::Marker::POINTS;
    marker_cells.action = visualization_msgs::msg::Marker::ADD;
    marker_cells.scale.x = 0.05;
    marker_cells.scale.y = 0.05;
    marker_cells.color.a = 1.0;

    // Markers for viewpoints
    marker_viewpoints.ns = "viewpoints";
    marker_viewpoints.id = 0;
    marker_viewpoints.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker_viewpoints.action = visualization_msgs::msg::Marker::ADD;
    marker_viewpoints.scale.x = 0.03;
    marker_viewpoints.scale.y = 0.03;
    marker_viewpoints.scale.z = 0.03;
    marker_viewpoints.color.a = 1.0;

    // Markers for the robot positions
    marker_robot_pos.ns = "top_robot_positions";
    marker_robot_pos.id = 0;
    marker_robot_pos.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker_robot_pos.action = visualization_msgs::msg::Marker::ADD;
    marker_robot_pos.scale.x = 0.05;
    marker_robot_pos.scale.y = 0.05;
    marker_robot_pos.scale.z = 0.05;
    marker_robot_pos.color.a = 1.0;

    // Markers for the field of view
    marker_fov.ns = "viewpoints_fov";
    marker_fov.id = 0;
    marker_fov.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker_fov.action = visualization_msgs::msg::Marker::ADD;
    marker_fov.scale.x = 0.01;  // line width
    marker_fov.color.r = 1.0;
    marker_fov.color.g = 1.0;
    marker_fov.color.b = 0.0;
    marker_fov.color.a = 0.8;
    int fov_num_points = 20;
    
    int i = 0;
    for (const auto& cluster : frontier_clusters) {
        // Generate a random color for this cluster, if not enough colors exist yet
        if (frontier_colors.size() <= i) {
            std::array<double, 3> color;
            do {
                color = {randomDouble(), randomDouble(), randomDouble()};
            } while (std::any_of(
                frontier_colors.begin(), frontier_colors.end(),
                [&color](const auto& used_color) { return used_color == color; }));
            frontier_colors.push_back(color);
        }
        const auto& color = frontier_colors[i];

        std::string frontier_str = "Frontier cluster " + std::to_string(++i) + ": ";
        frontier_str += std::to_string(cluster.cells_.size()) + " cells";
        frontier_str += " at average position (" + std::to_string(cluster.average_pos_.x()) + ", " + std::to_string(cluster.average_pos_.y()) + ") ";
        frontier_status.push_back(frontier_str);

        // Add cells of this cluster to the marker
        for (const auto& cell : cluster.cells_) {
            geometry_msgs::msg::Point point;
            Eigen::Vector2d pos;
            map.indexToPos(cell, pos);
            point.x = pos.x();
            point.y = pos.y();
            point.z = 0.0; // Assuming a 2D map
            marker_cells.points.push_back(point);

            std_msgs::msg::ColorRGBA point_color;
            point_color.r = color[0];
            point_color.g = color[1];
            point_color.b = color[2];
            point_color.a = 1.0;
            marker_cells.colors.push_back(point_color);

            // Display the top n viewpoints for this cluster (if available)
            int n = 4;
            for (int k = 0; k < std::min(n, static_cast<int>(cluster.viewpoints_.size())); ++k) {
                const auto& viewpoint = cluster.viewpoints_[k];
                geometry_msgs::msg::Point point;
                point.x = viewpoint.pos_[0];
                point.y = viewpoint.pos_[1];
                point.z = 0.0;
                marker_viewpoints.points.push_back(point);

                std_msgs::msg::ColorRGBA point_color;
                point_color.r = color[0];
                point_color.g = color[1];
                point_color.b = color[2];
                point_color.a = 1.0;
                marker_viewpoints.colors.push_back(point_color);

                // Draw FOV and robot position for top viewpoint
                if (k == 0) {
                    createFOVMarkers(marker_fov, viewpoint.pos_[0], viewpoint.pos_[1], viewpoint.yaw_);

                    geometry_msgs::msg::Point point_robot_pos;
                    point_robot_pos.x = viewpoint.robot_pos_[0];
                    point_robot_pos.y = viewpoint.robot_pos_[1];
                    point_robot_pos.z = 0.0;
                    marker_robot_pos.points.push_back(point_robot_pos);

                    std_msgs::msg::ColorRGBA point_robot_pos_color;
                    point_robot_pos_color.r = color[0];
                    point_robot_pos_color.g = color[1];
                    point_robot_pos_color.b = color[2];
                    point_robot_pos_color.a = 1.0;
                    marker_robot_pos.colors.push_back(point_robot_pos_color);
                }
            }
        }
    }
}

void VisualizationUtils::drawNextViewpoint(visualization_msgs::msg::Marker & marker_next_viewpoint,
        const Eigen::Vector2d & position, const double yaw) {
    marker_next_viewpoint.ns = "next_viewpoint";
    marker_next_viewpoint.id = 0;
    marker_next_viewpoint.type = visualization_msgs::msg::Marker::ARROW;
    marker_next_viewpoint.action = visualization_msgs::msg::Marker::ADD;
    
    marker_next_viewpoint.pose.position.x = position.x() - 0.2 * std::cos(yaw);
    marker_next_viewpoint.pose.position.y = position.y() - 0.2 * std::sin(yaw);
    marker_next_viewpoint.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    marker_next_viewpoint.pose.orientation.x = q.x();
    marker_next_viewpoint.pose.orientation.y = q.y();
    marker_next_viewpoint.pose.orientation.z = q.z();
    marker_next_viewpoint.pose.orientation.w = q.w();

    marker_next_viewpoint.scale.x = 0.4;
    marker_next_viewpoint.scale.y = 0.05;
    marker_next_viewpoint.scale.z = 0.05;
    marker_next_viewpoint.color.r = 0.0;
    marker_next_viewpoint.color.g = 1.0;
    marker_next_viewpoint.color.b = 0.0;
    marker_next_viewpoint.color.a = 0.5;
}

void VisualizationUtils::drawBox(visualization_msgs::msg::Marker & marker_box,
        const Eigen::Vector2i& bmin,
        const Eigen::Vector2i& bmax,
        const Map & map) {
    marker_box.points.clear();
    marker_box.ns = "box";
    marker_box.id = 0;
    marker_box.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker_box.action = visualization_msgs::msg::Marker::ADD;
    marker_box.scale.x = 0.01;  // line width
    marker_box.color.r = 0.0;
    marker_box.color.g = 0.0;
    marker_box.color.b = 1.0;
    marker_box.color.a = 0.8;
            
    geometry_msgs::msg::Point p_min, p_max;
    Eigen::Vector2d pos_min, pos_max;
    map.indexToPos(bmin, pos_min);
    p_min.x = pos_min.x();
    p_min.y = pos_min.y();
    p_min.z = 0.0;
    map.indexToPos(bmax, pos_max);
    p_max.x = pos_max.x();
    p_max.y = pos_max.y();
    p_max.z = 0.0;

    // Define the 4 corners of the box
    geometry_msgs::msg::Point p1 = p_min;
    geometry_msgs::msg::Point p2 = p_min; p2.x = p_max.x;
    geometry_msgs::msg::Point p3 = p_max;
    geometry_msgs::msg::Point p4 = p_min; p4.y = p_max.y;

    // Add lines between the corners to form the box
    marker_box.points.push_back(p1); marker_box.points.push_back(p2);
    marker_box.points.push_back(p2); marker_box.points.push_back(p3);
    marker_box.points.push_back(p3); marker_box.points.push_back(p4);
    marker_box.points.push_back(p4); marker_box.points.push_back(p1);
}

void VisualizationUtils::drawPath(visualization_msgs::msg::Marker & marker_path,
        const std::vector<Eigen::Vector2d> & path) {
    marker_path.points.clear();
    marker_path.ns = "path";
    marker_path.id = 0;
    marker_path.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker_path.action = visualization_msgs::msg::Marker::ADD;
    marker_path.scale.x = 0.02;  // line width
    marker_path.color.r = 1.0;
    marker_path.color.g = 0.0;
    marker_path.color.b = 0.0;
    marker_path.color.a = 1.0;

    for (const auto & point : path) {
        geometry_msgs::msg::Point p;
        p.x = point.x();
        p.y = point.y();
        p.z = 0.0;

        if (!marker_path.points.empty()) {
            const auto & last = marker_path.points.back();
            if (std::abs(last.x - p.x) < 1e-9 && std::abs(last.y - p.y) < 1e-9 &&
                std::abs(last.z - p.z) < 1e-9) {
                continue;
            }
        }

        marker_path.points.push_back(p);
    }
}

void VisualizationUtils::drawDistanceField(visualization_msgs::msg::Marker & marker_field,
        const Map & map, double max_dist) {
    marker_field.points.clear();
    marker_field.colors.clear();
    marker_field.ns = "distance_field";
    marker_field.id = 0;
    marker_field.type = visualization_msgs::msg::Marker::POINTS;
    marker_field.action = visualization_msgs::msg::Marker::ADD;
    marker_field.scale.x = 0.05;  // point width
    marker_field.scale.y = 0.05;  // point height

    int voxel_num_x = map.getVoxelNumX();
    int voxel_num_y = map.getVoxelNumY();

    for (int y = 0; y < voxel_num_y; ++y) {
        for (int x = 0; x < voxel_num_x; ++x) {
            Eigen::Vector2i id(x, y);
            Eigen::Vector2d pos;
            map.indexToPos(id, pos);

            if (!(map.getOccupancy(id) == Map::FREE))
                continue;

            double distance = map.getDistance(pos);

            geometry_msgs::msg::Point p;
            p.x = pos.x();
            p.y = pos.y();
            p.z = 0.0;

            marker_field.points.push_back(p);

            const double color_ratio = max_dist > 0.0
                ? std::clamp(distance / max_dist, 0.0, 1.0)
                : 1.0;
            std_msgs::msg::ColorRGBA color;
            color.r = 1.0 - color_ratio;
            color.g = color_ratio;
            color.b = 0.0;
            color.a = 1.0;
            marker_field.colors.push_back(color);
        }
    }
}

void VisualizationUtils::drawTarget(visualization_msgs::msg::Marker & marker_target,
        const Eigen::Vector2d & target_position) {
    marker_target.ns = "target";
    marker_target.id = 0;
    marker_target.type = visualization_msgs::msg::Marker::SPHERE;
    marker_target.action = visualization_msgs::msg::Marker::ADD;
    marker_target.scale.x = 0.05;
    marker_target.scale.y = 0.05;
    marker_target.scale.z = 0.05;
    marker_target.pose.position.x = target_position.x();
    marker_target.pose.position.y = target_position.y();
    marker_target.pose.position.z = 0.0;
    marker_target.pose.orientation.w = 1.0;
    marker_target.color.r = 1.0;
    marker_target.color.g = 1.0;
    marker_target.color.b = 0.0;
    marker_target.color.a = 1.0;
}

void VisualizationUtils::drawVelocity(visualization_msgs::msg::Marker & marker_velocity,
        const Eigen::Vector2d & position,
        const Eigen::Vector2d & velocity) {
    marker_velocity.ns = "robot_velocity";
    marker_velocity.id = 0;
    marker_velocity.type = visualization_msgs::msg::Marker::ARROW;
    marker_velocity.action = visualization_msgs::msg::Marker::ADD;

    double scale = 0.4 *velocity.norm() / 1.0;

    double yaw = std::atan2(velocity.y(), velocity.x());
    marker_velocity.pose.position.x = position.x() - (scale/2) * std::cos(yaw);
    marker_velocity.pose.position.y = position.y() - (scale/2) * std::sin(yaw);
    marker_velocity.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    marker_velocity.pose.orientation.x = q.x();
    marker_velocity.pose.orientation.y = q.y();
    marker_velocity.pose.orientation.z = q.z();
    marker_velocity.pose.orientation.w = q.w();

    marker_velocity.scale.x = scale;
    marker_velocity.scale.y = 0.05;
    marker_velocity.scale.z = 0.05;
    marker_velocity.color.r = 1.0;
    marker_velocity.color.g = 0.0;
    marker_velocity.color.b = 0.0;
    marker_velocity.color.a = 1.0;
}

void VisualizationUtils::createFOVMarkers(visualization_msgs::msg::Marker & marker_fov, 
        const double origin_x, const double origin_y, const double yaw) {
    geometry_msgs::msg::Point origin;
    origin.x = origin_x;
    origin.y = origin_y;
    origin.z = 0.0;

    // Sweep from the right edge to the left edge
    double start_angle = yaw + fov_left_angle_;
    double end_angle   = yaw + fov_right_angle_;

    geometry_msgs::msg::Point previous_point = origin;

    int fov_num_points = 20;

    for (int i = 0; i <= fov_num_points; ++i) {
        double angle = start_angle + (end_angle - start_angle) * i / fov_num_points;
        geometry_msgs::msg::Point p;
        p.x = origin.x + fov_max_dist_ * std::cos(angle);
        p.y = origin.y + fov_max_dist_ * std::sin(angle);
        p.z = 0.0;

        if (i == 0) {
            marker_fov.points.push_back(origin);
            marker_fov.points.push_back(p);
        } else {
            marker_fov.points.push_back(previous_point);
            marker_fov.points.push_back(p);
        }
        previous_point = p;
    }

    marker_fov.points.push_back(previous_point);
    marker_fov.points.push_back(origin);
}

double VisualizationUtils::randomDouble() {
    // returns random double between 0 and 1
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<double> dist(0.0, 1.0);
    return dist(gen);
}
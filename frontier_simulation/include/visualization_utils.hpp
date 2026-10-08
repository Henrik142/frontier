#pragma once

#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <Eigen/Eigen>
#include <random>
#include <algorithm>

#include "perception_utils.hpp"
#include "map.hpp"
#include "frontiers.hpp"

class VisualizationUtils {
public:
    VisualizationUtils();
    ~VisualizationUtils();

    void initVisUtils(const PerceptionUtils::InitFOVParams& fov_params);

    void drawRobotPose(visualization_msgs::msg::Marker & marker_pos,
        const Eigen::Vector2d & position, const double yaw);
    void drawRobotPose(visualization_msgs::msg::Marker & marker_pos,
        const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & pose_msg);
    void drawRobotFOV(visualization_msgs::msg::Marker & marker_fov,
        const geometry_msgs::msg::PoseStamped::ConstSharedPtr & camera_pose_msg);
    void drawRobotFOV(visualization_msgs::msg::Marker & marker_fov,
        const Eigen::Vector2d & camera_pos, const double yaw);
    void drawMap(nav_msgs::msg::OccupancyGrid & grid_msg, const Map & map,
        bool show_inflation = false);
    void drawFrontiers(visualization_msgs::msg::Marker & marker_cells,
        visualization_msgs::msg::Marker & marker_viewpoints,
        visualization_msgs::msg::Marker & marker_robot_pos,
        visualization_msgs::msg::Marker & marker_fov,
        std::list<std::string> & frontier_status,
        std::vector<std::array<double, 3>> & frontier_colors,
        const std::list<FrontierCluster> & frontier_clusters,
        const Map & map);
    void drawNextViewpoint(visualization_msgs::msg::Marker & marker_next_viewpoint,
        const Eigen::Vector2d & position, const double yaw);
    void drawBox(visualization_msgs::msg::Marker & marker_box,
        const Eigen::Vector2i& bmin,
        const Eigen::Vector2i& bmax,
        const Map & map);
    void drawPath(visualization_msgs::msg::Marker & marker_path,
        const std::vector<Eigen::Vector2d> & path);
    void drawDistanceField(visualization_msgs::msg::Marker & marker_field,
        const Map & map, double max_dist);
    void drawTarget(visualization_msgs::msg::Marker & marker_target,
        const Eigen::Vector2d & target_position);
    void drawVelocity(visualization_msgs::msg::Marker & marker_velocity,
        const Eigen::Vector2d & position,
        const Eigen::Vector2d & velocity);
        
private:
    void createFOVMarkers(visualization_msgs::msg::Marker& marker_fov, 
        const double origin_x, const double origin_y, const double yaw);
    double randomDouble();

    double fov_left_angle_, fov_right_angle_, fov_max_dist_;
};
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <Eigen/Eigen>
#include <chrono>

#include "astar.hpp"
#include "map.hpp"
#include "map_builder.hpp"
#include "visualization_utils.hpp"
#include "perception_utils.hpp"

class AstarTest : public rclcpp::Node
{
public:
    AstarTest();
    ~AstarTest();

private:
    void initParams();
    void testPoints();
    void mapUpdateCallback();
    int pathUpdate(Eigen::Vector2d start, Eigen::Vector2d goal,
                    std::vector<double> & durations,
                    std::vector<double> & path_lengths);
    void drawMarker(visualization_msgs::msg::Marker & msg_marker,
                    const std::vector<Eigen::Vector2d> & points,
                    const double & scale,
                    const std::vector<double> & color);
    double randomDouble();
    double seededRandomDouble();

    Map::InitMapParams map_params_;    
    PerceptionUtils::InitFOVParams fov_params_;
    Astar::InitAstarParams astar_params_;
    
    std::shared_ptr<Map> map_;
    std::shared_ptr<VisualizationUtils> vis_utils_;
    std::shared_ptr<Astar> astar_path_searcher_;
    
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_map_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_path_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_start_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_goal_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_visited_;

    rclcpp::TimerBase::SharedPtr map_update_timer_;
    bool test_points_run_ = false;
};
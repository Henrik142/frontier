#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include "visualization_utils.hpp"

class HippoVisualization : public rclcpp::Node
{
public:
    HippoVisualization();
    ~HippoVisualization();

private:
    void initParams();
    void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & odometry_msg);
    void positionTargetCallback(const geometry_msgs::msg::PointStamped::ConstSharedPtr & position_target_msg);
    
    std::string vehicle_name_;

    std::shared_ptr<VisualizationUtils> vis_utils_;
    std::shared_ptr<PerceptionUtils> percep_utils_;
    PerceptionUtils::InitFOVParams fov_params_;
    
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_robot_pose_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_fov_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_target_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odometry_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_position_target_;
};
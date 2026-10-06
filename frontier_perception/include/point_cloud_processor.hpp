#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <functional>

class PointCloudProcessor : public rclcpp::Node
{
public:
    PointCloudProcessor();
    ~PointCloudProcessor();

private:
    void pointCloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);

    std::string vehicle_name_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_pc_camera_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pc_map_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};


#include "point_cloud_processor.hpp"

PointCloudProcessor::PointCloudProcessor() : Node("point_cloud_processor") {
    vehicle_name_ = this->declare_parameter<std::string>("vehicle_name", "");

    // TF buffer + listener: subscribes to /tf and /tf_static in the background
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    publisher_pc_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_pointcloud", 10);

    subscription_pc_camera_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "rgbd_camera/points",
      rclcpp::SensorDataQoS(),   // sensor data QoS: best-effort, suited for high-rate streams
      std::bind(&PointCloudProcessor::pointCloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribed to rgbd_camera/points");
}

PointCloudProcessor::~PointCloudProcessor() {
}

void PointCloudProcessor::pointCloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
    // Target and source frame
    const std::string target_frame = "map";
    const std::string source_frame = vehicle_name_ + "/rgbd_camera_link";

    geometry_msgs::msg::TransformStamped transform;
    
    // get the transform from camera frame to map frame
    try
    {
      transform = tf_buffer_->lookupTransform(
        target_frame,                             // target frame
        source_frame,                             // source frame
        cloud_msg->header.stamp,                  // time stamp for the transform
        rclcpp::Duration::from_seconds(0.2));     // timeout for the transform
    }
    catch (const tf2::TransformException & ex)
    {
      RCLCPP_WARN(this->get_logger(), "Could not transform point cloud from '%s' to '%s': %s",
                  source_frame.c_str(), target_frame.c_str(), ex.what());
      return;
    }

    // Transform the PointCloud2 directly (still in ROS message form)
    sensor_msgs::msg::PointCloud2 cloud_world;
    tf2::doTransform(*cloud_msg, cloud_world, transform);

    // Convert ROS message to PCL point cloud for processing
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(cloud_world, *cloud);

    if (cloud->empty())
    {
      RCLCPP_WARN(this->get_logger(), "Received empty point cloud after transform, skipping");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Transformed cloud has %zu points in frame '%s'",
                cloud->size(), cloud_world.header.frame_id.c_str());

    // Set all z values to zero for easier visualization
    for (auto & point : cloud->points) {
      point.z = 0.0f;
    }

    /*
    // Example processing step: voxel grid downsampling
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(0.05f, 0.05f, 0.05f);  // 5cm voxels
    voxel_filter.filter(*filtered);

    RCLCPP_INFO(this->get_logger(), "Downsampled to %zu points", filtered->size());
    */
    // ... further processing (ground removal, clustering, etc.) goes here

    // Publish the point cloud
    pcl::toROSMsg(*cloud, cloud_world);
    publisher_pc_map_->publish(cloud_world);
}
#ifndef AGV_HARDWARE__SCAN_MERGER_V2_HPP_
#define AGV_HARDWARE__SCAN_MERGER_V2_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <chrono>
#include <mutex>
#include <memory>
#include <string>
#include <vector>


class ScanMergerV2 : public rclcpp::Node
{
public:
    //constructor
    ScanMergerV2(const rclcpp::NodeOptions & options);

private:
    void scan1_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void scan2_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    std::vector<std::pair<float, float>> laserscan_to_point(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg, std::string lidar_frame);
    void process_and_publish_scans();

    // initializing pointers and variables
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar1_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar2_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr merged_scan_;
    
    // Store latest scans with thread safety
    sensor_msgs::msg::LaserScan::SharedPtr scan1_;
    sensor_msgs::msg::LaserScan::SharedPtr scan2_;
    std::mutex scan_mutex_;

    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    float max_range_;
    float angle_min_;
    float angle_max_;
    float angle_increment_;
    int num_points_;

    

    // Track last processed timestamps to avoid duplicate processing
    rclcpp::Time last_scan1_time_;
    rclcpp::Time last_scan2_time_;

    std::string robot_namespace_;
    std::string robot_name_;
    std::string base_frame_;
    std::string lidar_right_frame_;
    std::string lidar_left_frame_;
    std::string scan1_topic_;
    std::string scan2_topic_;
};

#endif 
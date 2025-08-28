#ifndef AGV_HARDWARE__TAG_TRANSFORM_HPP_
#define AGV_HARDWARE__TAG_TRANSFORM_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/exceptions.h>
#include <memory>
#include <string>

namespace agv_hardware
{

class TagTransformNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for TagTransformNode
     * Initializes the node, sets up TF2 buffer and listener, creates publisher and timer
     */
    TagTransformNode(const rclcpp::NodeOptions & options);

private:
    /**
     * @brief Callback function to publish transform data
     * Gets transform from base frame to tag frame and publishes as PoseStamped
     */
    void publish_transform();
    
    // TF2 components
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Publisher and timer
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Frame names
    std::string base_frame_;
    std::string tag_frame_;
    std::string tag_frame_rotated_;
    std::string namespace_param;
    
    // Warning throttling
    double last_warning_time_;
    double warning_interval_;
};

}  // namespace agv_hardware

#endif  // AGV_HARDWARE__TAG_TRANSFORM_HPP_
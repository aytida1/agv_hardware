#include "agv_hardware/tag_transform.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace agv_hardware
{

TagTransformNode::TagTransformNode(const rclcpp::NodeOptions & options) : Node("tag_transform_node",options), last_warning_time_(0)
{
    // Initialize TF2 buffer and listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        
        
        // Parameters for frame names
        this->declare_parameter("base_frame", "map");
        this->declare_parameter("tag_frame", "tag36h11:0");
        this->declare_parameter("publish_rate", 15.0);
        this->declare_parameter("warning_interval", 5.0); // Log warning every 5 seconds
        this->declare_parameter("namespace", "");
        
        base_frame_ = this->get_parameter("base_frame").as_string();
        tag_frame_ = this->get_parameter("tag_frame").as_string();
        warning_interval_ = this->get_parameter("warning_interval").as_double();
        namespace_param = this->get_parameter("namespace").as_string();
        
        double rate = this->get_parameter("publish_rate").as_double();

        std::string topic_name = namespace_param + "/detected_dock_pose";
        // Create publisher for detected dock pose
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            topic_name, 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
            std::bind(&TagTransformNode::publish_transform, this));
        
        RCLCPP_INFO(this->get_logger(), "Tag Transform Node started");
        RCLCPP_INFO(this->get_logger(), "Publishing transform from %s to %s on /detected_dock_pose", 
                   base_frame_.c_str(), tag_frame_.c_str());
}

void TagTransformNode::publish_transform()
{
    try
    {
        // Get transform from base_link to tag
        geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped = tf_buffer_->lookupTransform(
                base_frame_, tag_frame_, tf2::TimePointZero);
            
            // Convert transform to pose stamped
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.stamp = this->now();
            pose_stamped.header.frame_id = base_frame_;
            
            // Set position from transform translation
            pose_stamped.pose.position.x = transform_stamped.transform.translation.x;
            pose_stamped.pose.position.y = transform_stamped.transform.translation.y;
            pose_stamped.pose.position.z = transform_stamped.transform.translation.z;
            
            // Set orientation from transform rotation
            pose_stamped.pose.orientation.x = transform_stamped.transform.rotation.x;
            pose_stamped.pose.orientation.y = transform_stamped.transform.rotation.y;
            pose_stamped.pose.orientation.z = transform_stamped.transform.rotation.z;
            pose_stamped.pose.orientation.w = transform_stamped.transform.rotation.w;
            
            // Publish the pose
            pose_publisher_->publish(pose_stamped);

           
            
            // RCLCPP_INFO(this->get_logger(), 
            //             "Published pose: x=%.3f, y=%.3f, z=%.3f", 
            //             pose_stamped.pose.position.x,
            //             pose_stamped.pose.position.y, 
            //             pose_stamped.pose.position.z);
        }
        catch (const tf2::TransformException & ex)
        {
            // Only log warning if enough time has passed since last warning
            double current_time = this->now().seconds();
            if (current_time - last_warning_time_ >= warning_interval_)
            {
                RCLCPP_WARN(this->get_logger(), 
                           "Could not transform %s to %s: %s (suppressing similar warnings for %.1f seconds)", 
                           tag_frame_.c_str(), base_frame_.c_str(), ex.what(), warning_interval_);
                last_warning_time_ = current_time;
            }
        }
    }
    
    // TF2 components
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Publisher and timer
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Frame names
    std::string base_frame_;
    std::string tag_frame_;
    
    // Warning throttling
    double last_warning_time_;
    double warning_interval_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(agv_hardware::TagTransformNode)

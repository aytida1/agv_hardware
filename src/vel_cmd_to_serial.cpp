#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <cmath>
#include <modbus/modbus.h>

class VelToSerial : public rclcpp::Node {
public:
    VelToSerial() : Node("vel_to_serial")
    {
        vel_topic_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel",
            10,
            std::bind(&VelToSerial::vel_callback, this, std::placeholders::_1)
        );
    

        // Initialize Modbus RTU connection
        modbus_ctx_ = modbus_new_rtu("/dev/ttyUSB0", 115200, 'N', 8, 1);
        if (modbus_ctx_ == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "Unable to create Modbus context");
            return;
        }
            
        // Set slave ID (default for ZLAC8015D is usually 1)
        modbus_set_slave(modbus_ctx_, 1);
            
        if (modbus_connect(modbus_ctx_) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Connection failed: %s", modbus_strerror(errno));
            modbus_free(modbus_ctx_);
            modbus_ctx_ = nullptr;
        } else {
            RCLCPP_INFO(this->get_logger(), "Modbus connection established");
        }
    }
    
    ~VelToSerial() {
        if (modbus_ctx_) {
            modbus_close(modbus_ctx_);
            modbus_free(modbus_ctx_);
        }
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_topic_;
    modbus_t *modbus_ctx_;
    
    void vel_callback(const geometry_msgs::msg::Twist msg)
    {
        double x_vel = msg.linear.x;
        double ang_vel = msg.angular.z;

        double l_vel = -(x_vel - ang_vel*wheel_dist/2);
        double r_vel = -(x_vel + ang_vel*wheel_dist/2);

        vel_l_rpm = (l_vel/wheel_radius)/(2*M_PI) * 60.0;
        vel_r_rpm = (r_vel/wheel_radius)/(2*M_PI) * 60.0;

        //send rpm command to ZLAC8015D
        send_motor_commands();
    }

    void send_motor_commands()
    {
        if (!modbus_ctx_) {
            RCLCPP_WARN(this->get_logger(), "Modbus context not available");
            return;
        }
        
        // Write left motor target velocity to register 0x2088
        uint16_t left_data = static_cast<uint16_t>(vel_l_rpm * 10);
        if (modbus_write_register(modbus_ctx_, 0x2088, left_data) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to write left motor command: %s", modbus_strerror(errno));
        }
        
        // Write right motor target velocity to register 0x2089
        uint16_t right_data = static_cast<uint16_t>(vel_r_rpm * 10);
        if (modbus_write_register(modbus_ctx_, 0x2089, right_data) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to write right motor command: %s", modbus_strerror(errno));
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Sent commands - Left: %f RPM (0x%04X), Right: %f RPM (0x%04X)", 
                    vel_l_rpm/10, left_data, vel_r_rpm/10, right_data);
    }


    double vel_l_rpm = 0;
    double vel_r_rpm = 0; 

    // robot parameters
    double wheel_radius = 0.105; // in meter
    double wheel_dist = 0.380; // in meter

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelToSerial>());
    rclcpp::shutdown();
    return 0;
}

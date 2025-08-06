#!/usr/bin/env python3
from pymodbus.client import ModbusSerialClient as ModbusClient
import ZLAC8015D
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster

import math
from nav_msgs.msg import Odometry

class VelToSerial(Node):
    def __init__(self):
        super().__init__('vel_to_serial')

        # Declare parameters with default value
        self.declare_parameter('namespace', '')
        self.declare_parameter('serial_port', '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B003LVFB-if00-port0')
        
        # Get parameter values
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        
        # Robot parameters
        self.wheel_radius = 0.0525  # wheel radius in meters (adjust as needed)
        self.wheel_dist = 0.38    # distance between wheels in meters (adjust as needed)
        self.subscription = self.create_subscription(
            Twist,
            f"{self.namespace}/cmd_vel",
            self.cmd_vel_callback,
            10)
        self.odom_publisher = self.create_publisher(Odometry, f"{self.namespace}/odom", 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        # self.client = ModbusClient(port='/dev/ttyUSB0', baudrate=115200, timeout=1)
        # self.client.connect()
        self.motors = ZLAC8015D.Controller(port=self.serial_port)

        self.motors.disable_motor()

        self.motors.set_accel_time(500,500)
        self.motors.set_decel_time(500,500)

        self.motors.set_mode(3)
        self.motors.enable_motor()

        

        # cmds = [140, 170]
        #cmds = [100, 50]
        #cmds = [150, -100]
        self.odom_timer = 0.067  # Hz
        self.timer = self.create_timer(self.odom_timer, self.timer_callback)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert Euler angles (roll, pitch, yaw) to quaternion (qx, qy, qz, qw).
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return [qx, qy, qz, qw]
        

    def cmd_vel_callback(self, msg):
        # Convert Twist message to motor commands using differential drive kinematics
        x_vel = msg.linear.x
        ang_vel = msg.angular.z
        
        # Calculate left and right wheel velocities
        l_vel = -(x_vel + ang_vel * self.wheel_dist / 2)
        r_vel = (x_vel - ang_vel * self.wheel_dist / 2)
        
        # Convert velocities to RPM
        vel_l_rpm = int((l_vel / (self.wheel_radius * 2 * math.pi)) * 60.0)
        vel_r_rpm = int((r_vel / (self.wheel_radius * 2 * math.pi)) * 60.0)
        
        # Send motor commands
        self.send_motor_commands(vel_l_rpm, vel_r_rpm)
        # self.get_logger().info(f'Sent RPM commands - Left: {vel_l_rpm:.2f}, Right: {vel_r_rpm:.2f}')
    
    def send_motor_commands(self, vel_l_rpm, vel_r_rpm):
        """Send RPM commands to ZLAC8015D motor controller"""
        self.motors.set_rpm(vel_l_rpm, vel_r_rpm)

    def timer_callback(self):
        rpmL, rpmR = self.motors.get_rpm()
        radpsL = -rpmL*2*math.pi/60
        radpsR = rpmR*2*math.pi/60

        #calculate distance travelled by each wheel
        left_distance = radpsL * self.wheel_radius * self.odom_timer
        right_distance = radpsR * self.wheel_radius * self.odom_timer

        delta_distance = (left_distance + right_distance) / 2.0
        delta_theta = (-right_distance + left_distance) / self.wheel_dist


        # update robot's position
        self.x += delta_distance * math.cos(self.theta)
        self.y += delta_distance * math.sin(self.theta)  
        self.theta += delta_theta

        # calculate linear and angular velocity of the robot
        linear_velocity = (self.wheel_radius / 2) * (radpsL + radpsR)
        angular_velocity = (self.wheel_radius / self.wheel_dist) * (radpsL - radpsR)

        # create odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = f"{self.namespace}/odom"
        odom_msg.child_frame_id = f"{self.namespace}/base_link"
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y

        # convert theta to quaternion
        quaternion = self.get_quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = quaternion[0]
        odom_msg.pose.pose.orientation.y = quaternion[1]
        odom_msg.pose.pose.orientation.z = quaternion[2]
        odom_msg.pose.pose.orientation.w = quaternion[3]

        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.angular.z = angular_velocity

        self.odom_publisher.publish(odom_msg)

        # create transform stamped message
        transform = TransformStamped()
        transform.header.stamp = odom_msg.header.stamp  # keep eye on it
        transform.header.frame_id = f"{self.namespace}/odom"
        transform.child_frame_id = f"{self.namespace}/base_link"

        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0

        transform.transform.rotation = odom_msg.pose.pose.orientation

        #publish the transform
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = VelToSerial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # node.client.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
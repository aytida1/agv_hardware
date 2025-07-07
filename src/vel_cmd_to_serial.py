#!/usr/bin/env python3
from pymodbus.client import ModbusSerialClient as ModbusClient
import ZLAC8015D
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class VelToSerial(Node):
    def __init__(self):
        super().__init__('vel_to_serial')
        
        # Robot parameters
        self.wheel_radius = 0.1  # wheel radius in meters (adjust as needed)
        self.wheel_dist = 0.5    # distance between wheels in meters (adjust as needed)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        # self.client = ModbusClient(port='/dev/ttyUSB0', baudrate=115200, timeout=1)
        # self.client.connect()
        self.motors = ZLAC8015D.Controller(port='/dev/ttyUSB0')

        self.motors.disable_motor()

        self.motors.set_accel_time(1000,1000)
        self.motors.set_decel_time(1000,1000)

        self.motors.set_mode(3)
        self.motors.enable_motor()

        # cmds = [140, 170]
        #cmds = [100, 50]
        #cmds = [150, -100]
        

    def cmd_vel_callback(self, msg):
        # Convert Twist message to motor commands using differential drive kinematics
        x_vel = msg.linear.x
        ang_vel = msg.angular.z
        
        # Calculate left and right wheel velocities
        l_vel = -(x_vel - ang_vel * self.wheel_dist / 2)
        r_vel = (x_vel + ang_vel * self.wheel_dist / 2)
        
        # Convert velocities to RPM
        vel_l_rpm = int((l_vel / self.wheel_radius) / (2 * math.pi) * 60.0)
        vel_r_rpm = int((r_vel / self.wheel_radius) / (2 * math.pi) * 60.0)
        
        # Send motor commands
        self.send_motor_commands(vel_l_rpm, vel_r_rpm)
        self.get_logger().info(f'Sent RPM commands - Left: {vel_l_rpm:.2f}, Right: {vel_r_rpm:.2f}')
    
    def send_motor_commands(self, vel_l_rpm, vel_r_rpm):
        """Send RPM commands to ZLAC8015D motor controller"""
        self.motors.set_rpm(vel_l_rpm, vel_r_rpm)

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
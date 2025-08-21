#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from agv_hardware.action import ControlServo

import time
from SM120BL_MB_motor_controller import Feetech_motor

class controlServoActionServer(Node):

    def __init__(self):
        super().__init__('control_servo_action_server')

        # Declare parameters with default value
        self.declare_parameter('namespace', '')
        self.declare_parameter('SERIAL_PORT', '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B003LKH9-if00-port0')

        # Get parameter values
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value
        serialPort = self.get_parameter('SERIAL_PORT').get_parameter_value().string_value

        action_name_string = f"{self.namespace}/ControlServo"
        self._action_server = ActionServer(
            self,
            ControlServo,
            action_name_string,
            self.execute_callback
        )

        # declaring motor instance
        self.motor = Feetech_motor(serial_port=serialPort)
        time.sleep(0.5)
        if not self.motor.client or not self.motor.client.is_open:
            self.get_logger().error("Failed to connect. Please check the port.")
        else:
            self.get_logger().info("Connection successful!")
        
        # setting velcity
        self.motor.set_motor_velocity(1, 20)
        self.motor.set_motor_velocity(2, 10)
        self.motor.set_motor_velocity(3, 10)

    # here motor_id : 1 for main lift servo
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        lift_and_lock = goal_handle.request.lift_and_lock

        # True then first lift up and then lock
        if lift_and_lock:
            self.lift_up()
            time.sleep(0.5)
            self.servo_lock()

        # False then first unlock and then lift down
        else:
            self.servo_unlock()
            time.sleep(0.5)
            self.lift_down()

        goal_handle.succeed()

        result = ControlServo.Result()
        result.success = True
        return result

    def lift_up(self):
        target = 16000
        self.motor.motor_torque_enable(1)
        time.sleep(0.1)
        
        self.motor.set_motor_position(1, target)
        while(self.motor.get_motor_position(1) < (target-5)):
            time.sleep(0.1)
            continue

        self.motor.motor_torque_disable(1)
        return 
    
    def lift_down(self):
        home = 1000
        self.motor.motor_torque_enable(1)
        time.sleep(0.1)
        
        self.motor.set_motor_position(1, home)
        while(self.motor.get_motor_position(1) > (home+5)):
            time.sleep(0.1)
            continue
        
        self.motor.motor_torque_disable(1)

    def servo_lock(self):
        servo2_target = 1300
        servo3_target = 4000

        self.motor.motor_torque_enable(2)
        self.motor.motor_torque_enable(3)
        time.sleep(0.1)

        self.motor.set_motor_position(2, servo2_target)
        self.motor.set_motor_position(3, servo3_target)
        while((self.motor.get_motor_position(2) < (servo2_target-5)) and (self.motor.get_motor_position(3) < (servo3_target-5))):
            time.sleep(0.1)
            continue
        
        self.motor.motor_torque_disable(2)
        self.motor.motor_torque_disable(3)

    def servo_unlock(self):
        servo2_home = 300
        servo3_home = 3000

        self.motor.motor_torque_enable(2)
        self.motor.motor_torque_enable(3)
        time.sleep(0.1)

        self.motor.set_motor_position(2, servo2_home)
        self.motor.set_motor_position(3, servo3_home)
        while((self.motor.get_motor_position(2) > (servo2_home+5)) and (self.motor.get_motor_position(3) > (servo3_home+5))):
            time.sleep(0.1)
            continue
        
        self.motor.motor_torque_disable(2)
        self.motor.motor_torque_disable(3)
        
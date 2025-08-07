#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener

import math

class GracefulAlgo(Node):
    # constructor
    def __init__(self):
        super().__init__('graceful_algo')
        # member parameters
        self.namespace = "agv1"
        self.timer_sec = 0.1
        self.target_frame = self.namespace + "/base_link"
        self.source_frame = "tag36h11:0"

        # dynamic parameters
        self.declare_parameter('k_one', value=1.0)
        self.declare_parameter('k_two', value=1.0)
        self.declare_parameter('heading_vel', value=0.07)
        
        # subscribe to cmd_vel topic 
        topic_name = self.namespace + "/cmd_vel"
        self.vel_pub = self.create_publisher(
            Twist,
            topic_name,
            10
        )

        # for getting transform
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # create a timer to call a get_transform method
        self.timer = self.create_timer(self.timer_sec, self.graceful_method)

    
    # Main method to calculate angular velocity
    def graceful_method(self):
        
        # handle not getting transform pose
        try:
            t = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().error(f"Could not transform {self.source_frame} and {self.target_frame}: {ex}")
            stop_cmd = Twist()
            stop_cmd.linear.x = 0.0
            stop_cmd.angular.z = 0.0
            self.vel_pub.publish(stop_cmd)
            return
        
        # considering apriltag as origin
        x = t.transform.translation.x
        y = t.transform.translation.y
        q = t.transform.rotation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

        # th(theta) : angle between common line and x axis of apriltag
        th = -1.0*math.atan2(y, x)

        # r : distance between two frames
        r = math.sqrt(x*x + y*y)

        # dl(delta) : angle between common line and x axis of robot
        dl = th + yaw

        # k1 : is the ratio of rate of change in th to the rate of change in r
        k1 = self.get_parameter('k_one').get_parameter_value().double_value

        # k2 : as per formula it will make omega more reactive
        k2 = self.get_parameter('k_two').get_parameter_value().double_value

        # v : heading velocity of robot in x axis
        v = self.get_parameter('heading_vel').get_parameter_value().double_value

        ############ main formula for gracefull controller ################
        part1 = k2*(dl - math.atan(-k1*th))
        part2 = (1 + k1/(1 + (k1*th)**2))*math.sin(dl)
        
        omega = -1.0*(part1 + part2)*v/r
        ###################################################################

        # prepare and publish twist message to topic
        cmd = Twist()

        # distance threshold of 0.45
        if r <= 0.45:
            cmd.linear.x = 0.0
        else: 
            cmd.linear.x = v
            cmd.angular.z = omega

        self.vel_pub.publish(cmd)




def main(args=None):
    rclpy.init(args=args)
    # make an instance
    my_node_class = GracefulAlgo()
    rclpy.spin(my_node_class)

    my_node_class.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
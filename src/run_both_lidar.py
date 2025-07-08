#!/usr/bin/env python3

import rclpy
from rclpy.node import Node




class BasicNode(Node):
    def __init__(self):
        super().__init__('run_both_lidar')
        self.get_logger().info('-----------Lidar Nodes has been initiated-----------')

def main(args=None):
    rclpy.init(args=args)
    node = BasicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
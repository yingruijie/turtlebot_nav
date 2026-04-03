import sys
import rclpy
import numpy as np
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class TurtlebotNavNode(Node):
    def __init__(self, ):
        super().__init__('turtlebot_nav')
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pub_cmd_vel = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.waypoints = np.asarray([[0.3, 0.2], [3.0, 0.1]])

    def odom_callback(self, msg:Odometry):
        pass

    def scan_callback(self, msg:LaserScan):
        pass

    def navigate(self,):
        pass

def main(args=None):
    if args is None:
        args = sys.argv
    rclpy.init(args=args)
    turtlebot_nav_node = TurtlebotNavNode()
    try:
        rclpy.spin(turtlebot_nav_node)
    except KeyboardInterrupt:
        pass
    turtlebot_nav_node.destroy_node()
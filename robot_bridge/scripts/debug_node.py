#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import math
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from rclpy.constants import S_TO_NS
from sensor_msgs.msg import Imu
import tf_transformations
from tf2_ros import TransformBroadcaster

class debug_node(Node):
    def __init__(self):
        super().__init__('debug_node')

        # self.imu_data_publisher = self.create_publisher(Imu, '/imu_raw', 10)
        self.odom_publisher = self.create_publisher(Float32MultiArray,'/wheel_vel',10) 


        self.imu_sent_timer = self.create_timer(0.1, self.timer_callback)

        self.dz = 0

    def timer_callback(self):
        pass



def main(args=None):
    rclpy.init(args=args)
    node = debug_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
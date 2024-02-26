#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math
import numpy as np

class sensor_calibration(Node):
    def __init__(self):
        super().__init__('sensor_calibration')
        self.create_subscription(Float32MultiArray, '/BKG_imu_raw', self.imu_callback, 10)
        self.imu_raw = [0.0,0.0,0.0,0.0,0.0,0.0]
        # self.iteration = 

    def imu_callback(self, msg):
        self.imu_raw = msg.data

def main(args=None):
    rclpy.init(args=args)
    node = sensor_calibration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()


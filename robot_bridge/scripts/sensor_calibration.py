#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math
import numpy as np

class sensor_calibration(Node):
    def __init__(self):
        super().__init__('sensor_calibration')
        self.create_subscription(Float32MultiArray, '/BGK_imu_raw', self.imu_callback, 10)
        self.imu_raw = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.iteration = 10
        self.reset()

    def reset(self):
        self.arr_angular_x = []
        self.arr_angular_y = []
        self.arr_angular_z = []
        self.arr_linear_x = []
        self.arr_linear_y = []
        self.arr_linear_z = []
        self.count = 0
        self.use_linear = False

    def imu_callback(self, msg):
        if self.count == self.iteration:
            self.calculate_calibration()
            self.reset()
        self.get_logger().info(f"-------{self.count}")
        self.use_linear = True
        self.arr_angular_x.append(msg.data[3])
        self.arr_angular_y.append(msg.data[4])
        self.arr_angular_z.append(msg.data[5])
        self.arr_linear_x.append(msg.data[0])
        self.arr_linear_y.append(msg.data[1])
        self.arr_linear_z.append(msg.data[2])
        self.count += 1

    def calculate_calibration(self):
        print("-" * 20)
        x = np.array(self.arr_angular_x)
        y = np.array(self.arr_angular_y)
        z = np.array(self.arr_angular_z)
        # calculate mean
        x_mean = np.mean(x)
        y_mean = np.mean(y)
        z_mean = np.mean(z)
        print("Angular Offset")
        print(f"- {x_mean}\n- {y_mean}\n- {z_mean}")
        # calculate covariance
        xyz = np.stack((x, y, z), axis=0)
        cov = np.cov(xyz)
        print(f"Angular Covariance")
        print(cov)
        if self.use_linear:
            x = np.array(self.arr_linear_x)
            y = np.array(self.arr_linear_y)
            z = np.array(self.arr_linear_z)
            # calculate mean
            x_mean = np.mean(x)
            y_mean = np.mean(y)
            z_mean = np.mean(z)
            print("Linear Offset")
            print(f"- {x_mean}\n- {y_mean}\n- {z_mean}")
            # calculate covariance
            xyz = np.stack((x, y, z), axis=0)
            cov = np.cov(xyz)
            print(f"Linear Covariance")
            print(cov)


def main(args=None):
    rclpy.init(args=args)
    node = sensor_calibration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()


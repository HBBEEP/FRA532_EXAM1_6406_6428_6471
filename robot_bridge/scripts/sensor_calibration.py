#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math
import numpy as np
from ament_index_python.packages import get_package_share_directory
import os
import yaml


class sensor_calibration(Node):
    def __init__(self):
        super().__init__('sensor_calibration')
        self.create_subscription(Float32MultiArray, '/BGK_imu_raw', self.imu_callback, 10)
        self.iteration = 10
        self.reset()

    def reset(self):
        acc_mean_cov = {  
            'mean': [0.0, 0.0, 0.0],
            'covariance': [
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0]
            ]
        }

        gyro_mean_cov = {
            'mean': [0.0, 0.0, 0.0],
            'covariance': [
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0]
            ]
        }

        self.data = {'acc': acc_mean_cov, 'gyro': gyro_mean_cov}

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
            self.save_yaml_file(self.data)
            # self.reset()
        # self.get_logger().info(f"-------{self.count}")
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
        self.data['gyro']['mean'][0] = float(x_mean)
        self.data['gyro']['mean'][1] = float(y_mean)
        self.data['gyro']['mean'][2] = float(z_mean)

        for i in range(3): 
            for j in range(3):
                print(f'---> {i*3 , +j}')
                self.data['gyro']['covariance'][i][j] = float(cov[i][j])
                

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
            self.data['acc']['mean'][0] = float(x_mean)
            self.data['acc']['mean'][1] = float(y_mean)
            self.data['acc']['mean'][2] = float(z_mean)
            for i in range(3): 
                for j in range(3):                
                    self.data['acc']['covariance'][i][j] = float(cov[i][j])



    
    def save_yaml_file(self, data):
        package_share_directory = get_package_share_directory('robot_bridge')
        parts = package_share_directory.split(os.path.sep)
        cleaned_package_share_directory = os.path.sep.join(parts[:-4])
        yaml_file_path = os.path.join(cleaned_package_share_directory, 'src/robot_bridge/config', 'test.yaml')
        with open(yaml_file_path, 'w') as yaml_file:
            yaml.dump(data, yaml_file, default_flow_style=False)
        print(f"Data saved to {yaml_file_path}")


def main(args=None):
    rclpy.init(args=args)
    node = sensor_calibration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()

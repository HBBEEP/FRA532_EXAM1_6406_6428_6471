#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import yaml
import os
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

class test_file(Node):
    def __init__(self):
        super().__init__('test_file')
        # Example data
        self.data = {
            'name': 'John',
            'age': 30,
            'city': 'New York'
        }

        # Path to the YAML file

        package_share_directory = get_package_share_directory('robot_bridge')
        print(package_share_directory)
        parts = package_share_directory.split(os.path.sep)

        # Remove the last three parts of the path
        cleaned_package_share_directory = os.path.sep.join(parts[:-4])

        self.yaml_file_path = os.path.join(cleaned_package_share_directory, 'src/robot_bridge/config', 'test.yaml')

        # self.yaml_file_path = '/robot_bridge/config/test.yaml'
        self.save_to_yaml(self.data, self.yaml_file_path )

    # Function to save data to YAML file
    def save_to_yaml(self, data, yaml_file_path):
        with open(yaml_file_path, 'w') as yaml_file:
            yaml.dump(data, yaml_file, default_flow_style=False)
        print(f"Data saved to {yaml_file_path}")


def main(args=None):
    rclpy.init(args=args)
    node = test_file()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
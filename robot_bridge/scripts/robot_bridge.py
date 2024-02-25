#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math
import numpy as np
from rclpy.constants import S_TO_NS


class robot_bridge(Node):
    def __init__(self):
        super().__init__('robot_bridge')
        self.create_subscription(Float32MultiArray, '/BGK_wheel_vel', self.wheel_vel_callback, 10)
        self.robot_position = [0.0, 0.0, 0.0] # x, y, theta
        self.robot_vel = [0.0, 0.0]
        self.robot_twist = [0.0, 0.0]
        self.WHEEL_RADIUS = 0.03375
        self.WHEEL_SEPARATION = 0.16480
 
    def wheel_vel_callback(self, msg):
        self.robot_vel =  msg.data        
        self.robot_twist = self.forward_kinematics(self.robot_vel[0], self.robot_vel[1])
        self.calculate_wheel_odometry()
        self.get_logger().info(f"X : {self.robot_position[0]} | Y : {self.robot_position[1]} | Z : {self.robot_position[2]}")
        
    def calculate_wheel_odometry(self):
        dt = self.get_clock().now() - self.prev_time
        dx = self.robot_twist[0] * math.cos(self.robot_position[2] ) * (dt.nanoseconds / S_TO_NS)
        dy = self.robot_twist[0] * math.sin(self.robot_position[2] ) * (dt.nanoseconds / S_TO_NS)
        dtheta = self.robot_twist[1] * (dt.nanoseconds / S_TO_NS)
        self.robot_position[0] += dx
        self.robot_position[1] += dy
        self.robot_position[2] += dtheta
        self.robot_position[2] = math.atan2(math.sin(self.robot_position[2]), math.cos(self.robot_position[2]))
        self.prev_time = self.get_clock().now()
    

    def forward_kinematics(self, rwheel_velocity, lwheel_velocity):
        fk_constant = self.WHEEL_RADIUS * np.array([[0.5,0.5],
                                [1/self.WHEEL_SEPARATION, -1/self.WHEEL_SEPARATION]])
        wheel_velocity = np.array([rwheel_velocity, lwheel_velocity])
        robot_twist = np.dot(fk_constant, wheel_velocity)
        robot_twist[0] = 0 if abs(robot_twist[0]) < 0.0001 else robot_twist[0] 
        robot_twist[1] = 0 if abs(robot_twist[1]) < 0.0001 else robot_twist[1] 
        return robot_twist

def main(args=None):
    rclpy.init(args=args)
    node = robot_bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
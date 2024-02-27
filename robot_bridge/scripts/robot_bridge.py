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

class robot_bridge(Node):
    def __init__(self):
        super().__init__('robot_bridge')
        self.create_subscription(Float32MultiArray, '/BGK_wheel_vel', self.wheel_vel_callback, 10)
        self.create_subscription(Float32MultiArray, '/BGK_imu_raw', self.imu_raw_callback, 10)

        self.imu_data_publisher = self.create_publisher(Imu, '/imu/data', 10)
        self.odom_publisher = self.create_publisher(Odometry,'/wheel/odom',10) 


        self.imu_sent_timer = self.create_timer(0.1, self.timer_callback)

        self.robot_position = [0.0, 0.0, 0.0] # x, y, theta
        self.robot_vel = [0.0, 0.0]
        self.robot_twist = [0.0, 0.0]
        self.imu_raw = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.WHEEL_RADIUS = 0.03375
        self.WHEEL_SEPARATION = 0.16480

        self.lx_offset = 0.0
        self.ly_offset = 0.0
        self.lz_offset = 0.0

        self.gx_offset = 0.0
        self.gy_offset = 0.0
        self.gz_offset = 0.0

        self.acc_cov = []
        self.gyro_cov = []
        self.load_yaml_file()
        self.odom_broadcaster = TransformBroadcaster(self)

        self.prev_time = self.get_clock().now()

    def timer_callback(self):
        self.imu_data_pub()

    def imu_raw_callback(self, msg):
        self.imu_raw = msg.data 

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
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.robot_position[0] 
        t.transform.translation.y = self.robot_position[1] 
        t.transform.translation.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, self.robot_position[2])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.odom_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        
        # set the position
        odom.pose.pose.position.x = self.robot_position[0] 
        odom.pose.pose.position.y = self.robot_position[1] 
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # set the velocity
        odom.child_frame_id = "base_footprint"
        odom.twist.twist.linear.x = self.robot_twist[0]
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0

        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.robot_twist[1]

        odom.pose.covariance = [ 0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
        odom.twist.covariance = [ 0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
        
        # measure covariance  น้อย e-9 เยอะ 100 

        self.odom_publisher.publish(odom)

    def forward_kinematics(self, rwheel_velocity, lwheel_velocity):
        fk_constant = self.WHEEL_RADIUS * np.array([[0.5,0.5],
                                [1/self.WHEEL_SEPARATION, -1/self.WHEEL_SEPARATION]])
        wheel_velocity = np.array([rwheel_velocity, lwheel_velocity])
        robot_twist = np.dot(fk_constant, wheel_velocity)
        robot_twist[0] = 0 if abs(robot_twist[0]) < 0.0001 else robot_twist[0] 
        robot_twist[1] = 0 if abs(robot_twist[1]) < 0.0001 else robot_twist[1] 
        return robot_twist
    


    def imu_data_pub(self):
        imu_msg = Imu()
        imu_msg.linear_acceleration.x = float(self.imu_raw[0]) - self.lx_offset
        imu_msg.linear_acceleration.y = float(self.imu_raw[1]) - self.ly_offset
        imu_msg.linear_acceleration.z = float(self.imu_raw[2]) - self.lz_offset
        imu_msg.linear_acceleration_covariance = self.acc_cov

        # Gyroscope data in rad/s
        imu_msg.angular_velocity.x = float(self.imu_raw[3]) - self.gx_offset
        imu_msg.angular_velocity.y = float(self.imu_raw[4]) - self.gy_offset
        imu_msg.angular_velocity.z = float(self.imu_raw[5]) - self.gz_offset
        imu_msg.angular_velocity_covariance = self.gyro_cov
        self.imu_data_publisher.publish(imu_msg)        


    def load_yaml_file(self):
        package_share_directory = get_package_share_directory('robot_bridge')
        parts = package_share_directory.split(os.path.sep)
        cleaned_package_share_directory = os.path.sep.join(parts[:-4])
        yaml_file_path = os.path.join(cleaned_package_share_directory, 'src/robot_bridge/config', 'test.yaml')
        with open(yaml_file_path, 'r') as yaml_file:
            config = yaml.safe_load(yaml_file)

        self.lx_offset = config['acc']['mean'][0]
        self.ly_offset = config['acc']['mean'][1]
        self.lz_offset = config['acc']['mean'][2]

        self.gx_offset = config['gyro']['mean'][0]
        self.gy_offset = config['gyro']['mean'][1]
        self.gz_offset = config['gyro']['mean'][2]

        for i in range(3):
            for j in range(3):
                self.acc_cov.append(float(config['acc']['covariance'][i][j]))
                self.gyro_cov.append(float(config['gyro']['covariance'][i][j]))



def main(args=None):
    rclpy.init(args=args)
    node = robot_bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from geometry_msgs.msg import Twist
from rclpy.constants import S_TO_NS
import math
import tf_transformations
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import csv

class robot_controller(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.robot_timer = self.create_timer(0.05, self.timer_callback)
        self.robot_twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(Int8, '/robot_command', self.robot_command_callback, 10)

        self.prev_time = self.get_clock().now()
        self.prev_time_control = self.get_clock().now()

        self.robot_twist = [0.0, 0.0]
        self.robot_position = [0.0, 0.0, 0.0]
        self.odom_broadcaster = TransformBroadcaster(self)
        self.command = 0
        self.prev_command = None
        self.run_flag = False
        self.log_wheel = []
    def timer_callback(self):


        if (self.command == 0):
            self.get_logger().info("==== IDLE STATE ====")
            self.publish_robot_twist(0.0, 0.0)

        if (self.command == 1):
            self.get_logger().info("==== SQUARE ====")
            self.rectangle_path()
        if (self.command == 2):
            # self.get_logger().info("==== CIRCLE ====")
            self.circle_path()
        if (self.command == 3):
            # self.get_logger().info("==== CIRCLE ====")
            self.linear_path()
        
        if (self.command == 4):
            self.circle_around_path()

        self.calculate_wheel_odometry()
        self.prev_command = self.command

    def inverse_kinematics(self, linear_velocity, angular_velocity ):
        ik_constant = (1/self.WHEEL_RADIUS)* np.array([[1, 0.5 * self.WHEEL_SEPARATION],
                                [1 , -0.5 *self.WHEEL_SEPARATION]])
        robot_twist = np.array([linear_velocity, angular_velocity])
        wheel_velocity = np.dot(ik_constant, robot_twist)
        return wheel_velocity

    def robot_command_callback(self, msg):
        if (msg.data != self.command):
            self.get_logger().info("==== RECIEVE COMMAND ====")
            self.command = msg.data
            self.prev_time_control = self.get_clock().now() 


    def publish_robot_twist(self, linear_vel, angular_vel):
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.robot_twist = [linear_vel, angular_vel]
        self.robot_twist_publisher.publish(twist)
        wheel_velocity = self.inverse_kinematics(linear_vel, angular_vel)
        self.log_wheel.append(wheel_velocity)

    def calculate_wheel_odometry(self):
        dt = self.get_clock().now() - self.prev_time

        dx = self.robot_twist[0] * math.cos(self.robot_position[2] ) * (dt.nanoseconds / S_TO_NS)
        dy = self.robot_twist[0] * math.sin(self.robot_position[2] ) * (dt.nanoseconds / S_TO_NS)
        dtheta = self.robot_twist[1] * (dt.nanoseconds / S_TO_NS)

        self.robot_position[0] += dx
        self.robot_position[1] += dy
        self.robot_position[2] += dtheta
        self.robot_position[2] = math.atan2(math.sin(self.robot_position[2]), math.cos(self.robot_position[2]))
        self.get_logger().info(f"X : {self.robot_position[0]} | Y : {self.robot_position[1]} | Z : {self.robot_position[2]}")

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'cmd'
        t.transform.translation.x = self.robot_position[0] 
        t.transform.translation.y = self.robot_position[1] 
        t.transform.translation.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, self.robot_position[2])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.odom_broadcaster.sendTransform(t)
        
        self.prev_time = self.get_clock().now() 

    def rectangle_path(self):
        spent_time = (self.get_clock().now().nanoseconds - self.prev_time_control.nanoseconds)/S_TO_NS
        easy_gain = 1.0

        if (spent_time > 0 and spent_time < 10):
            
            self.publish_robot_twist(0.0, -0.157 * easy_gain)
        elif (spent_time > 10 and spent_time < 20):
            self.publish_robot_twist(0.1 * easy_gain, 0.0)

        elif (spent_time > 20 and spent_time < 30):
            self.publish_robot_twist(0.0, -0.157 * easy_gain)

        elif (spent_time > 30 and spent_time < 40):
            self.publish_robot_twist(0.1 * easy_gain, 0.0)

        elif (spent_time > 40 and spent_time < 50):
            self.publish_robot_twist(0.0, -0.157 * easy_gain)

        elif (spent_time > 50 and spent_time < 60):
            self.publish_robot_twist(0.1 * easy_gain, 0.0)

        elif (spent_time > 60 and spent_time < 70):
            self.publish_robot_twist(0.0, -0.157 * easy_gain)

        elif (spent_time > 70 and spent_time < 80):
            self.publish_robot_twist(0.1 * easy_gain, 0.0)

        elif (spent_time > 80 and spent_time < 90):
            self.publish_robot_twist(0.0, -0.157 * easy_gain)

        else:
            self.publish_robot_twist(0.0, 0.0)
 

    def circle_path(self):
        spent_time = (self.get_clock().now().nanoseconds - self.prev_time_control.nanoseconds)/S_TO_NS
        if (spent_time > 0 and spent_time < 100):
            self.publish_robot_twist(0.2, -0.1)
        else:
            self.publish_robot_twist(0.0, 0.0)

    def linear_path(self):
        spent_time = (self.get_clock().now().nanoseconds - self.prev_time_control.nanoseconds)/S_TO_NS
        if (spent_time > 0 and spent_time < 10):
            self.publish_robot_twist(0.1, 0.0)
        else:
            self.publish_robot_twist(0.0, 0.0)
            self.write_to_csv(self.log_wheel)

    def circle_around_path(self):
        spent_time = (self.get_clock().now().nanoseconds - self.prev_time_control.nanoseconds)/S_TO_NS
        if (spent_time > 0 and spent_time < 1000):
            self.publish_robot_twist(0.5, 0.5)
        else:
            self.publish_robot_twist(0.0, 0.0)

    # def circle_around_path(self):
    #     spent_time = (self.get_clock().now().nanoseconds - self.prev_time_control.nanoseconds)/S_TO_NS
    #     if (spent_time > 0 and spent_time < 1000):
    #         self.publish_robot_twist(0.5, 0.5)
    #     else:
    #         self.publish_robot_twist(0.0, 0.0)
            
    def write_to_csv(file_path, data, headers=None):
        try:
            with open(file_path, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                if headers:
                    writer.writerow(headers)
                writer.writerows(data)
            print("Data successfully written to", file_path)
        except Exception as e:
            print("An error occurred:", e)


def main(args=None):
    rclpy.init(args=args)
    node = robot_controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()

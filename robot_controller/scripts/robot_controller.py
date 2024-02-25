#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math
from rclpy.constants import S_TO_NS


class robot_controller(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.robot_timer = self.create_timer(0.5, self.timer_callback)
        self.robot_twist_publisher = self.create_publisher(Twist, '/BGK_cmd_vel', 10)
        self.create_subscription(Float64MultiArray, '/BGK_wheel_vel', self.wheel_vel_callback 10)


    def wheel_vel_callback(self, msg):
        msg.data = []
        

    def timer_callback(self):
        self.get_logger().info("publish robot twist")
        self.publish_robot_twist(50.0,100.0)

    def publish_robot_twist(self, linear_vel, angular_vel):
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.robot_twist_publisher.publish(twist)

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
    
def main(args=None):
    rclpy.init(args=args)
    node = robot_controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()

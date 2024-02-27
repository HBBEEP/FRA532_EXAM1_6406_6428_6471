#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.constants import S_TO_NS
import math
import tf_transformations
from geometry_msgs.msg import TransformStamped


class robot_controller(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.robot_timer = self.create_timer(0.1, self.timer_callback)
        self.robot_twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.prev_time = self.get_clock().now() 
        self.robot_twist = [0.0, 0.0]


    def timer_callback(self):
        if ((self.get_clock().now().nanoseconds - self.prev_time.nanoseconds)/S_TO_NS < 5):
            self.get_logger().info("publish robot twist")
            self.publish_robot_twist(1.0,0.0)
        else:
            self.publish_robot_twist(0.0,0.0)
            self.get_logger().info("===END=== (Robot should not move (may be))")
    
    def publish_robot_twist(self, linear_vel, angular_vel):
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.robot_twist = [linear_vel, angular_vel]
        self.robot_twist_publisher.publish(twist)
        self.calculate_wheel_odometry()

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



def main(args=None):
    rclpy.init(args=args)
    node = robot_controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()

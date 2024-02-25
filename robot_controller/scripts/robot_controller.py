#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.constants import S_TO_NS


class robot_controller(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.robot_timer = self.create_timer(0.5, self.timer_callback)
        self.robot_twist_publisher = self.create_publisher(Twist, '/BGK_cmd_vel', 10)
        self.prev_time = self.get_clock().now() 


    def timer_callback(self):
        if ((self.get_clock().now().nanoseconds - self.prev_time.nanoseconds)/S_TO_NS < 5):
            self.get_logger().info("publish robot twist")
            self.publish_robot_twist(1.0,0.0)
        else:
            self.publish_robot_twist(1.0,0.0)
            self.get_logger().info("===END=== (Robot should not move (may be))")

    def publish_robot_twist(self, linear_vel, angular_vel):
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.robot_twist_publisher.publish(twist)



def main(args=None):
    rclpy.init(args=args)
    node = robot_controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()

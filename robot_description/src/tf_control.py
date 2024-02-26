#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math
import numpy as np
from rclpy.constants import S_TO_NS
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException, TransformBroadcaster
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, Pose ,Pose, TransformStamped, Twist, PoseStamped
import tf2_ros


class tf_control(Node):
    def __init__(self):
        super().__init__('tf_control')
        self.create_subscription(Float32MultiArray, '/BGK_odom', self.odom_callback, 10)
        self.robot_position = [0.0, 0.0, 0.0] # x, y, theta

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.publish_transform = TransformBroadcaster(self)
 
    def odom_callback(self, msg):
        robot_position = msg.data
        self.pub_transform(robot_position[0],robot_position[1],robot_position[2])

    def pub_transform(self,X,Y,Z):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_footprint"
        t.child_frame_id = "base_link"

        t.transform.translation.x = X
        t.transform.translation.y = Y
        t.transform.rotation.z = Z

        self.publish_transform.sendTransform(t)
        
    
def main(args=None):
    rclpy.init(args=args)
    node = tf_control()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
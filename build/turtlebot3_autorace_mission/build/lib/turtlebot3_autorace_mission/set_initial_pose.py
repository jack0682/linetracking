#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

class SetInitialPose(Node):
    def __init__(self):
        super().__init__('set_initial_pose')
        
        # Create publisher for initial pose
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/initialpose', 
            10
        )
        
        # Wait a moment for publisher to be ready
        self.create_timer(1.0, self.publish_initial_pose)
        
    def publish_initial_pose(self):
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set initial position (로봇의 현재 위치)
        initial_pose.pose.pose.position.x = -2.47
        initial_pose.pose.pose.position.y = 1.67
        initial_pose.pose.pose.position.z = -0.00143
        
        # Set initial orientation (현재 방향)
        yaw = 0.0  # radians
        initial_pose.pose.pose.orientation.x = 0.0
        initial_pose.pose.pose.orientation.y = 0.0
        initial_pose.pose.pose.orientation.z = math.sin(yaw / 2.0)
        initial_pose.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        # Set covariance (uncertainty)
        initial_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909]
        
        # Publish initial pose
        self.initial_pose_pub.publish(initial_pose)
        self.get_logger().info('Initial pose set: x=-2.47, y=1.67')
        
        # Shutdown after publishing
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = SetInitialPose()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
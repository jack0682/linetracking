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
        
        # Set initial position (실제 시작 위치 - odom에서 확인한 좌표)
        initial_pose.pose.pose.position.x = -1.7603088878461883  # 진짜 시작 위치
        initial_pose.pose.pose.position.y = -0.18501192976186576
        initial_pose.pose.pose.position.z = 0.008508788515588466
        
        # Set initial orientation (실제 시작 방향)
        yaw = -1.556020  # radians (navigation_trigger에서 사용하는 방향)
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
        self.get_logger().info(f'Initial pose set: x={initial_pose.pose.pose.position.x:.3f}, y={initial_pose.pose.pose.position.y:.3f}, yaw={yaw:.3f}')
        
        # Shutdown after publishing
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = SetInitialPose()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
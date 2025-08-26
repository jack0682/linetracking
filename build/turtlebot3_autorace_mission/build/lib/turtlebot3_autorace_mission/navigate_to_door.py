#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import math

class NavigateToDoor(Node):
    def __init__(self):
        super().__init__('navigate_to_door')
        self.navigator = BasicNavigator()
        
        # Wait for navigation to fully activate
        self.navigator.waitUntilNav2Active()
        
        # Set goal pose (adjust coordinates based on your map)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        
        # Set target position (문 위치)
        goal_pose.pose.position.x = -0.751  # X 좌표
        goal_pose.pose.position.y = -0.0338  # Y 좌표
        goal_pose.pose.position.z = -0.00143
        
        # Set target orientation (문을 향하는 방향)
        yaw = 0.0  # radians
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        # Navigate to goal
        self.navigator.goToPose(goal_pose)
        
        # Monitor navigation
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(f'Distance to goal: {feedback.distance_remaining:.2f}m')
        
        result = self.navigator.getResult()
        if result == BasicNavigator.TaskResult.SUCCEEDED:
            self.get_logger().info('Successfully reached the door!')
        else:
            self.get_logger().info('Failed to reach the door')

def main(args=None):
    rclpy.init(args=args)
    navigator = NavigateToDoor()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
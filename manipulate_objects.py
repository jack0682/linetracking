#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState, GetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Twist, Vector3
import math

class ObjectManipulator(Node):
    def __init__(self):
        super().__init__('object_manipulator')
        
        # Create service clients
        self.set_state_client = self.create_client(SetEntityState, '/set_entity_state')
        self.get_state_client = self.create_client(GetEntityState, '/get_entity_state')
        
        # Wait for services
        while not self.set_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Set state service not available, waiting...')
        
        while not self.get_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Get state service not available, waiting...')

    def move_object(self, entity_name, x, y, z, roll=0, pitch=0, yaw=0):
        """Move object to specific position"""
        request = SetEntityState.Request()
        
        state = EntityState()
        state.name = entity_name
        
        # Set position
        state.pose.position.x = float(x)
        state.pose.position.y = float(y)
        state.pose.position.z = float(z)
        
        # Convert RPY to quaternion
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        state.pose.orientation.w = cy * cp * cr + sy * sp * sr
        state.pose.orientation.x = cy * cp * sr - sy * sp * cr
        state.pose.orientation.y = sy * cp * sr + cy * sp * cr
        state.pose.orientation.z = sy * cp * cr - cy * sp * sr
        
        request.state = state
        future = self.set_state_client.call_async(request)
        return future

    def apply_force(self, entity_name, force_x, force_y, force_z):
        """Apply velocity to object"""
        request = SetEntityState.Request()
        
        state = EntityState()
        state.name = entity_name
        
        # Set velocity (simulates force application)
        state.twist.linear.x = float(force_x)
        state.twist.linear.y = float(force_y)
        state.twist.linear.z = float(force_z)
        
        request.state = state
        future = self.set_state_client.call_async(request)
        return future

    def get_object_pose(self, entity_name):
        """Get current object pose"""
        request = GetEntityState.Request()
        request.name = entity_name
        future = self.get_state_client.call_async(request)
        return future

    def rotate_object_continuously(self, entity_name, angular_velocity_z):
        """Apply continuous rotation"""
        request = SetEntityState.Request()
        
        state = EntityState()
        state.name = entity_name
        
        # Set angular velocity
        state.twist.angular.z = float(angular_velocity_z)
        
        request.state = state
        future = self.set_state_client.call_async(request)
        return future

def main():
    rclpy.init()
    manipulator = ObjectManipulator()
    
    try:
        # Example: Move an existing object
        object_name = "red_box"  # Assumes object exists
        
        print(f"Moving {object_name} to position (2, 2, 1)...")
        future1 = manipulator.move_object(object_name, 2.0, 2.0, 1.0)
        rclpy.spin_until_future_complete(manipulator, future1)
        
        import time
        time.sleep(2)
        
        print(f"Rotating {object_name}...")
        future2 = manipulator.move_object(object_name, 2.0, 2.0, 1.0, 0, 0, math.pi/4)
        rclpy.spin_until_future_complete(manipulator, future2)
        
        time.sleep(2)
        
        print(f"Applying force to {object_name}...")
        future3 = manipulator.apply_force(object_name, 1.0, 0.0, 0.0)
        rclpy.spin_until_future_complete(manipulator, future3)
        
        print("Manipulation complete!")
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        manipulator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
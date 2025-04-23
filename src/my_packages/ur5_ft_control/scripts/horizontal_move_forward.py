#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, TwistStamped
from tf2_geometry_msgs import do_transform_pose
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import numpy as np
import sys
import threading
import time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState


class HorizontalMover(Node):
    """
    A ROS2 node that moves the UR5 end effector horizontally along its own axis
    using direct control interfaces instead of MoveIt.
    """

    def __init__(self):
        super().__init__('horizontal_mover')
        
        # Parameters
        self.declare_parameter('distance', 0.1)  # Distance to move in meters
        self.declare_parameter('speed_factor', 0.5)  # Speed factor (0.0-1.0)
        self.declare_parameter('direction', 'y')  # Direction in end effector frame: x, y, or z
        self.declare_parameter('joint_trajectory_topic', '/scaled_joint_trajectory_controller/joint_trajectory')  # Joint trajectory topic
        self.declare_parameter('duration', 2.0)  # Duration of movement in seconds
        self.declare_parameter('twist_topic', '/twist_controller/twist_cmd')  # Optional twist controller topic
        self.declare_parameter('use_twist_controller', False)  # Whether to use twist controller or joint trajectory
        self.declare_parameter('forward_position_topic', '/forward_position_controller/commands')
        
        # Get parameters
        self.distance = self.get_parameter('distance').value
        self.speed_factor = self.get_parameter('speed_factor').value
        self.direction = self.get_parameter('direction').value.lower()
        self.joint_trajectory_topic = self.get_parameter('joint_trajectory_topic').value
        self.duration = self.get_parameter('duration').value
        self.twist_topic = self.get_parameter('twist_topic').value
        self.use_twist_controller = self.get_parameter('use_twist_controller').value
        self.forward_position_topic = self.get_parameter('forward_position_topic').value
        
        # Validate direction parameter
        if self.direction not in ['x', 'y', 'z']:
            self.get_logger().error('Invalid direction parameter. Must be x, y, or z.')
            rclpy.shutdown()
            return
            
        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Callback groups
        self.joint_state_callback_group = MutuallyExclusiveCallbackGroup()
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()
        
        # Current joint state
        self.current_joint_positions = None
        self.joint_names = []
        
        # Joint state subscriber
        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.joint_state_callback_group
        )
        
        # Publishers
        self.joint_trajectory_publisher = self.create_publisher(
            JointTrajectory,
            self.joint_trajectory_topic,
            10
        )
        
        # Create twist publisher if using twist control
        if self.use_twist_controller:
            self.twist_publisher = self.create_publisher(
                TwistStamped,
                self.twist_topic,
                10
            )
        
        # Publisher for forward_position_controller
        self.forward_position_publisher = self.create_publisher(
            JointState,
            self.forward_position_topic,
            10
        )
        
        self.is_controlling = False
        self.get_logger().info('Waiting for joint states...')
        
        # Start movement after a short delay to allow initialization
        self.timer = self.create_timer(1.0, self.start_movement)
        
    def start_movement(self):
        """Start the movement once after initialization"""
        # Cancel the timer so this only executes once
        self.timer.cancel()
        
        # Make sure we have joint states
        if not self.current_joint_positions or len(self.joint_names) == 0:
            self.get_logger().warn('No joint states received yet, waiting...')
            self.timer = self.create_timer(1.0, self.start_movement)
            return
            
        self.horizontal_move()
        
    def joint_state_callback(self, msg):
        """Callback function for processing joint state readings."""
        self.current_joint_positions = msg.position
        self.joint_names = msg.name
        
    def get_current_pose(self):
        """Get the current pose of the end-effector in base frame"""
        try:
            # Wait for the transform to be available
            transform = self.tf_buffer.lookup_transform(
                'base_link',  # target frame
                'tool0',      # source frame (UR5 end effector frame)
                rclpy.time.Time(),
                rclpy.time.Duration(seconds=1.0)
            )
            
            # Create a dummy PoseStamped in the source frame
            p = PoseStamped()
            p.header.frame_id = 'tool0'
            p.header.stamp = self.get_clock().now().to_msg()
            p.pose.orientation.w = 1.0
            self.get_logger().info(f"PoseStamped: f{p.pose}")
            # Transform to base frame
            pose_base = do_transform_pose(p.pose, transform)
            
            return pose_base
            
        except Exception as e:
            self.get_logger().error(f'Failed to get current pose: {str(e)}')
            self.get_logger().error(f'Exception type: {type(e).__name__}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            return None
    
    def get_tool_transform(self):
        """Get the transform from base_link to tool0"""
        try:
            # Wait for the transform to be available
            transform = self.tf_buffer.lookup_transform(
                'base_link',  # target frame
                'tool0',      # source frame (UR5 end effector frame)
                rclpy.time.Time(),
                rclpy.time.Duration(seconds=1.0)
            )
            return transform
            
            
        except Exception as e:
            self.get_logger().error(f'Failed to get transform: {str(e)}')
            return None
    
    def horizontal_move(self):
        """Move the end effector horizontally along its own axis"""
        if self.is_controlling:
            self.get_logger().warn('Already executing a movement, skipping request')
            return
            
        self.is_controlling = True
        
        # Get current pose
        current_pose = self.get_current_pose()
        if current_pose is None:
            self.get_logger().error('Failed to get current pose')
            self.is_controlling = False
            return
            
        # Get tool transform to determine tool orientation in base frame
        tool_transform = self.get_tool_transform()
        if tool_transform is None:
            self.get_logger().error('Failed to get tool transform')
            self.is_controlling = False
            return
            
        # Calculate movement direction in base frame based on tool orientation
        movement_vector = np.zeros(3)
        
        # Set the direction
        if self.direction == 'x':
            # Extract the x axis of the tool frame in base frame coordinates
            movement_vector = np.array([
                tool_transform.transform.rotation.w**2 + tool_transform.transform.rotation.x**2 - 
                tool_transform.transform.rotation.y**2 - tool_transform.transform.rotation.z**2,
                2 * (tool_transform.transform.rotation.x * tool_transform.transform.rotation.y + 
                     tool_transform.transform.rotation.w * tool_transform.transform.rotation.z),
                2 * (tool_transform.transform.rotation.x * tool_transform.transform.rotation.z - 
                     tool_transform.transform.rotation.w * tool_transform.transform.rotation.y)
            ])
        elif self.direction == 'y':
            # Extract the y axis of the tool frame in base frame coordinates
            movement_vector = np.array([
                2 * (tool_transform.transform.rotation.x * tool_transform.transform.rotation.y - 
                     tool_transform.transform.rotation.w * tool_transform.transform.rotation.z),
                tool_transform.transform.rotation.w**2 - tool_transform.transform.rotation.x**2 + 
                tool_transform.transform.rotation.y**2 - tool_transform.transform.rotation.z**2,
                2 * (tool_transform.transform.rotation.y * tool_transform.transform.rotation.z + 
                     tool_transform.transform.rotation.w * tool_transform.transform.rotation.x)
            ])
        elif self.direction == 'z':
            # Extract the z axis of the tool frame in base frame coordinates
            movement_vector = np.array([
                2 * (tool_transform.transform.rotation.x * tool_transform.transform.rotation.z + 
                     tool_transform.transform.rotation.w * tool_transform.transform.rotation.y),
                2 * (tool_transform.transform.rotation.y * tool_transform.transform.rotation.z - 
                     tool_transform.transform.rotation.w * tool_transform.transform.rotation.x),
                tool_transform.transform.rotation.w**2 - tool_transform.transform.rotation.x**2 - 
                tool_transform.transform.rotation.y**2 + tool_transform.transform.rotation.z**2
            ])
            
        # Normalize vector
        norm = np.linalg.norm(movement_vector)
        if norm > 0:
            movement_vector = movement_vector / norm
        else:
            self.get_logger().error('Failed to calculate movement direction (zero norm)')
            self.is_controlling = False
            return
            
        # Scale by distance
        movement_vector *= self.distance
        
        self.get_logger().info(f'Moving in direction {self.direction} by {self.distance} meters')
        self.get_logger().info(f'Movement vector: {movement_vector}')
        
        # Calculate velocity
        velocity_vector = movement_vector / self.duration
        
        # Choose control method based on configuration
        if self.use_twist_controller:
            self.send_twist_command(velocity_vector)
        else:
            self.send_joint_trajectory(movement_vector)
            
    def send_twist_command(self, velocity_vector):
        """Send a twist command to move in the specified direction"""
        self.get_logger().info('Using twist controller for direct movement')
        
        # Create twist message
        twist_msg = TwistStamped()
        twist_msg.header.frame_id = 'base_link'
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Set linear velocity
        twist_msg.twist.linear.x = velocity_vector[0]
        twist_msg.twist.linear.y = velocity_vector[1]
        twist_msg.twist.linear.z = velocity_vector[2]
        
        # Publish the twist command
        self.twist_publisher.publish(twist_msg)
        
        # Create a timer to stop the movement after duration
        self.get_logger().info(f'Movement will complete in {self.duration} seconds...')
        self.stop_timer = self.create_timer(self.duration, self.stop_movement)
        
    def stop_movement(self):
        """Stop the movement by sending zero velocity"""
        if self.use_twist_controller:
            # Send zero twist
            zero_twist = TwistStamped()
            zero_twist.header.frame_id = 'base_link'
            zero_twist.header.stamp = self.get_clock().now().to_msg()
            self.twist_publisher.publish(zero_twist)
            
        self.is_controlling = False
        self.get_logger().info('Movement complete')
        
        # Cancel the timer
        self.stop_timer.cancel()
        
        # Shutdown after a short delay
        self.create_timer(1.0, lambda: rclpy.shutdown())
        
    def send_joint_trajectory(self, movement_vector):
        """Send a joint state command to move in the specified direction using forward_position_controller"""
        self.get_logger().info('Using forward_position_controller for direct movement')

        if len(self.joint_names) == 0 or self.current_joint_positions is None:
            self.get_logger().error('No joint states received')
            self.is_controlling = False
            return

        # Calculate a simple differential kinematics approximation
        scale_factor = 0.5

        # Copy current positions
        new_positions = list(self.current_joint_positions)

        xy_magnitude = np.sqrt(movement_vector[0]**2 + movement_vector[1]**2)
        if xy_magnitude > 0:
            angle = np.arctan2(movement_vector[1], movement_vector[0])
            new_positions[0] += scale_factor * xy_magnitude * np.cos(angle - new_positions[0])

        if abs(movement_vector[2]) > 0:
            new_positions[1] += scale_factor * movement_vector[2] * 0.7
            new_positions[2] += scale_factor * movement_vector[2] * 0.3

        # Publish the new joint positions as a JointState message
        js_msg = JointState()
        js_msg.header.stamp = self.get_clock().now().to_msg()
        js_msg.name = self.joint_names
        js_msg.position = new_positions

        self.forward_position_publisher.publish(js_msg)
        self.get_logger().info(f'JointState command sent, movement will complete in {self.duration} seconds')

        # Create timer to set control flag to false after movement completes
        self.stop_timer = self.create_timer(self.duration + 0.5, self.movement_complete)
    
    def movement_complete(self):
        """Called when movement is complete"""
        self.is_controlling = False
        self.get_logger().info('Movement complete')
        
        # Cancel the timer
        self.stop_timer.cancel()
        
        # Shutdown after a short delay
        self.create_timer(1.0, lambda: rclpy.shutdown())


def main(args=None):
    rclpy.init(args=args)
    
    # Parse command line arguments
    if len(sys.argv) > 1:
        direction = sys.argv[1].lower()
        if direction not in ['x', 'y', 'z']:
            print(f"Error: Direction must be x, y, or z. Got '{direction}'")
            if rclpy.ok():
                rclpy.shutdown()
            return
        
        # Override rclpy parameters with command line arguments
        args = []
        args.append('--ros-args')
        args.append('-p')
        args.append(f'direction:={direction}')
        
        if len(sys.argv) > 2:
            try:
                distance = float(sys.argv[2])
                args.append('-p')
                args.append(f'distance:={distance}')
            except ValueError:
                print(f"Error: Distance must be a number. Got '{sys.argv[2]}'")
                if rclpy.ok():
                    rclpy.shutdown()
                return
    
    horizontal_mover = HorizontalMover()
    
    # Create a multithreaded executor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(horizontal_mover)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        horizontal_mover.get_logger().info('Shutting down horizontal mover node...')
        executor.shutdown()
        horizontal_mover.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
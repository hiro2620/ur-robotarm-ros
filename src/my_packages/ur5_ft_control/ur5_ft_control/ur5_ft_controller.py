#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading

from robotiq_ft_sensor_interfaces.msg import FTSensor
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

# MoveIt2のPythonインターフェースをインポート
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.action import MoveGroup, ExecuteTrajectory


class UR5FTController(Node):
    """
    A ROS2 node that controls a UR5 robot based on FT300 torque sensor readings using MoveIt2.
    
    This node:
    1. Subscribes to FT300 sensor data
    2. Processes force and torque values
    3. Controls UR5 movements using MoveIt2 interface based on sensor readings
    """

    def __init__(self):
        super().__init__('ur5_ft_controller')
        
        # Initialize parameters with default values
        self.declare_parameter('force_threshold', 10.0)  # N
        self.declare_parameter('torque_threshold', 1.0)  # Nm
        self.declare_parameter('position_scaling', 0.001)  # Scale factor for position commands (m/N)
        self.declare_parameter('rotation_scaling', 0.01)  # Scale factor for rotation commands (rad/Nm)
        self.declare_parameter('planning_group', 'ur_manipulator')  # MoveIt2プランニンググループ名
        
        # Get parameter values
        self.force_threshold = self.get_parameter('force_threshold').value
        self.torque_threshold = self.get_parameter('torque_threshold').value
        self.position_scaling = self.get_parameter('position_scaling').value
        self.rotation_scaling = self.get_parameter('rotation_scaling').value
        self.planning_group = self.get_parameter('planning_group').value
        
        # コールバックグループの作成
        self.ft_callback_group = MutuallyExclusiveCallbackGroup()
        self.moveit_callback_group = MutuallyExclusiveCallbackGroup()
        
        # Create subscription to FT300 sensor data
        self.ft_subscription = self.create_subscription(
            FTSensor,
            '/robotiq_force_torque_sensor_broadcaster/wrench',  # Default topic for FT300 sensor readings
            self.ft_sensor_callback,
             10,  # QoS profile depth
            callback_group=self.ft_callback_group
        )
        
        # TF2リスナーの設定
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # MoveIt2 Actionクライアントの設定
        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            '/move_action',
            callback_group=self.moveit_callback_group
        )
        
        self.execute_trajectory_client = ActionClient(
            self,
            ExecuteTrajectory,
            '/execute_trajectory',
            callback_group=self.moveit_callback_group
        )
        
        # クライアントの準備ができるまで待機
        timeout = 5.0  # seconds
        if not self.move_group_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error('MoveGroup action server not available')
            return
            
        if not self.execute_trajectory_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error('ExecuteTrajectory action server not available')
            return
        
        # Initialize variables to store sensor readings
        self.fx, self.fy, self.fz = 0.0, 0.0, 0.0  # Force values in N
        self.tx, self.ty, self.tz = 0.0, 0.0, 0.0  # Torque values in Nm
        
        # スレッドの互換性のためのロック
        self.ft_lock = threading.Lock()
        
        # 現在のエンドエフェクタポーズを保存
        self.current_pose = None
        
        # 最後に力を検出した時間
        self.last_force_time = self.get_clock().now()
        
        # 制御中かどうかのフラグ
        self.is_controlling = False
        
        self.get_logger().info('UR5 FT300 Controller with MoveIt2 has been initialized')

    def ft_sensor_callback(self, msg):
        """Callback function for processing FT300 sensor readings."""
        with self.ft_lock:
            # Store force readings
            self.fx = msg.fx
            self.fy = msg.fy
            self.fz = msg.fz
            
            # Store torque readings
            self.tx = msg.mx
            self.ty = msg.my
            self.tz = msg.mz
        
        # Process readings and control robot
        self.process_ft_data()
        
        # Log data (for debugging)
        self.get_logger().debug(
            f'FT data - Force: [{self.fx:.2f}, {self.fy:.2f}, {self.fz:.2f}] N, '
            f'Torque: [{self.tx:.2f}, {self.ty:.2f}, {self.tz:.2f}] Nm'
        )

    def get_current_pose(self):
        """Get the current pose of the end-effector in base frame"""
        try:
            # Wait for the transform to be available
            transform = self.tf_buffer.lookup_transform(
                'base_link',  # target frame
                'tool0',      # source frame (UR5のエンドエフェクタフレーム)
                rclpy.time.Time(),
                rclpy.time.Duration(seconds=1.0)
            )
            
            # Create a dummy PoseStamped in the source frame
            p = PoseStamped()
            p.header.frame_id = 'tool0'
            p.header.stamp = self.get_clock().now().to_msg()
            p.pose.orientation.w = 1.0
            
            # Transform to base frame
            pose_base = do_transform_pose(p, transform)
            return pose_base.pose
            
        except Exception as e:
            self.get_logger().error(f'Failed to get current pose: {str(e)}')
            return None

    def send_moveit_goal(self, target_pose):
        """Send a goal to MoveIt2 to move to the target pose"""
        if self.is_controlling:
            self.get_logger().debug('Already executing a movement, skipping this request')
            return
            
        self.is_controlling = True
        
        # Create MoveIt2 goal
        goal_msg = MoveGroup.Goal()
        
        # Set the planning group
        goal_msg.request.group_name = self.planning_group
        
        # Set the pose target
        goal_msg.request.pose_stamped.header.frame_id = 'base_link'
        goal_msg.request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        goal_msg.request.pose_stamped.pose = target_pose
        
        # Set planning parameters
        goal_msg.request.num_planning_attempts = 1
        goal_msg.request.allowed_planning_time = 1.0  # 計画時間を短く設定
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5
        
        # Send the goal and wait for result
        self.move_group_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.move_group_feedback_callback
        ).add_done_callback(self.move_group_response_callback)

    def move_group_feedback_callback(self, feedback_msg):
        """MoveGroup action feedback callback"""
        feedback = feedback_msg.feedback
        self.get_logger().debug(f'MoveIt feedback: {feedback.state}')

    def move_group_response_callback(self, future):
        """MoveGroup action response callback"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('MoveIt goal rejected')
            self.is_controlling = False
            return

        self.get_logger().info('MoveIt goal accepted')
        
        # Get result future for this goal
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.move_group_result_callback)

    def move_group_result_callback(self, future):
        """MoveGroup action result callback"""
        result = future.result().result
        error_code = result.error_code.val
        
        if error_code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info('MoveIt planning succeeded!')
            
            # Execute the planned trajectory
            goal_msg = ExecuteTrajectory.Goal()
            goal_msg.trajectory = result.planned_trajectory
            
            self.execute_trajectory_client.send_goal_async(
                goal_msg
            ).add_done_callback(self.execute_trajectory_response_callback)
        else:
            self.get_logger().error(f'MoveIt planning failed with error code: {error_code}')
            self.is_controlling = False

    def execute_trajectory_response_callback(self, future):
        """ExecuteTrajectory action response callback"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Trajectory execution rejected')
            self.is_controlling = False
            return

        self.get_logger().info('Trajectory execution accepted')
        
        # Get result future for this goal
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.execute_trajectory_result_callback)

    def execute_trajectory_result_callback(self, future):
        """ExecuteTrajectory action result callback"""
        result = future.result().result
        error_code = result.error_code.val
        
        if error_code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info('Trajectory execution succeeded!')
        else:
            self.get_logger().error(f'Trajectory execution failed with error code: {error_code}')
            
        # Control is complete
        self.is_controlling = False

    def process_ft_data(self):
        """Process force/torque data and control the UR5 robot accordingly using MoveIt2."""
        # Calculate force and torque magnitudes
        force_magnitude = np.sqrt(self.fx**2 + self.fy**2 + self.fz**2)
        torque_magnitude = np.sqrt(self.tx**2 + self.ty**2 + self.tz**2)
        
        # Check if force or torque exceed thresholds
        if (force_magnitude > self.force_threshold or torque_magnitude > self.torque_threshold) and not self.is_controlling:
            self.last_force_time = self.get_clock().now()
            
            # 現在のエンドエフェクタのポーズを取得
            current_pose = self.get_current_pose()
            if current_pose is None:
                self.get_logger().error('Failed to get current pose of end-effector')
                return
                
            # 新しいターゲットポーズを計算（力/トルクに基づく）
            target_pose = Pose()
            
            # 現在の位置をコピー
            target_pose.position.x = current_pose.position.x
            target_pose.position.y = current_pose.position.y
            target_pose.position.z = current_pose.position.z
            
            # 現在の姿勢をコピー
            target_pose.orientation.x = current_pose.orientation.x
            target_pose.orientation.y = current_pose.orientation.y
            target_pose.orientation.z = current_pose.orientation.z
            target_pose.orientation.w = current_pose.orientation.w
            
            # 力とトルクに基づいて位置と姿勢を変更
            # 力に対する対応：力の向きとは逆方向に移動（コンプライアンス動作）
            target_pose.position.x -= self.fx * self.position_scaling
            target_pose.position.y -= self.fy * self.position_scaling
            target_pose.position.z -= self.fz * self.position_scaling
            
            # トルクに対する対応：本格的な姿勢制御をする場合はクォータニオン計算が必要だが、
            # 簡略化のため、トルクの影響は無視するか、別の方法で対応する必要がある
            
            # MoveIt2にゴールを送信
            self.send_moveit_goal(target_pose)
            
            self.get_logger().info(
                f'Sending robot to new target based on force/torque: '
                f'Position: [{target_pose.position.x:.4f}, {target_pose.position.y:.4f}, {target_pose.position.z:.4f}]'
            )


def main(args=None):
    rclpy.init(args=args)
    
    ur5_ft_controller = UR5FTController()
    
    # Create a multithreaded executor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(ur5_ft_controller)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        ur5_ft_controller.get_logger().info('Shutting down UR5 FT300 Controller node...')
        executor.shutdown()
        ur5_ft_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
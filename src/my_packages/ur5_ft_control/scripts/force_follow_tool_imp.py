#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header
from std_srvs.srv import Trigger
import numpy as np

TARGET_FORCE_Z = 5.0  # ツール軸方向（Z軸）の目標押し付け力[N]
MIN_FORCE = 0.5
MAX_FORCE = 20.0
MAX_VELOCITY = 0.2
FORCE_ERR_TORLERANCE = TARGET_FORCE_Z*0.1  # 力の誤差許容範
IMPEDANCE_GAIN = -1.0 # Gain for impedance control (velocity proportional to force error)

class ForceFollowNode(Node):
    def __init__(self):
        super().__init__('force_follow_node')
        self.subscription = self.create_subscription(
            WrenchStamped,
            '/force_torque_sensor_broadcaster/wrench',
            self.wrench_callback,
            100)
        self.twist_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 100)
        self.initial_force = None  # 初期値を保存
        self.initial_frame_id = None  # 初期ツール姿勢のframe_idを保存

        # /servo_node/start_servoサービスを呼び出す
        client = self.create_client(Trigger, '/servo_node/start_servo')
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/servo_node/start_servo service not available!')
        else:
            req = Trigger.Request()
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if future.result() is not None and future.result().success:
                self.get_logger().info('Successfully called /servo_node/start_servo service.')
            else:
                self.get_logger().error('Failed to call /servo_node/start_servo service.')

        self.get_logger().info('ForceFollowNode started with impedance control.')

    def wrench_callback(self, msg):
        fx = msg.wrench.force.x
        fy = msg.wrench.force.y
        fz = msg.wrench.force.z

        current_force = np.array([fx, fy, fz])
        if self.initial_force is None:
            self.initial_force = current_force
            self.initial_frame_id = msg.header.frame_id  # 初回のframe_idを保存
            self.get_logger().info(f'Initial force and frame_id set: {self.initial_frame_id}')
            return

        # 現在の力（オフセット補正済み）
        measured_force = current_force - self.initial_force

        # ツール軸（Z軸）方向の押し付け力
        # frame_idが変わらない前提で、Z軸方向成分を抽出
        force_tool_z = measured_force[2]  # ツールZ軸方向

        force_tool_z_abs = abs(force_tool_z)
        self.get_logger().info(f'Tool Z force: {force_tool_z:.3f} (abs: {force_tool_z_abs:.3f})')

        # 押し付け力が小さすぎる/大きすぎる場合は停止
        if force_tool_z_abs < MIN_FORCE or force_tool_z_abs > MAX_FORCE:
            self.get_logger().info('Tool Z force is too low or too high, stopping.')
            self.send_velocity_command(np.array([0.0, 0.0, 0.0]))
            return

        # 目標押し付け力との差分（絶対値で制御）
        force_error = TARGET_FORCE_Z - force_tool_z_abs

        # 目標押し付け力との差分が小さい場合は停止
        if abs(force_error) < FORCE_ERR_TORLERANCE:
            self.get_logger().info('Force error is small, stopping.')
            self.send_velocity_command(np.array([0.0, 0.0, 0.0]))
            return

        # 目標方向（現在の押し付け方向）に速度指令を出す (Impedance Control)
        direction = np.sign(force_tool_z) if force_tool_z != 0 else 1.0
        velocity_cmd_z = IMPEDANCE_GAIN * force_error * direction # Impedance control
        self.get_logger().info(f'Force error: {force_error:.3f}, velocity_cmd_z: {velocity_cmd_z:.3f}')

        # 安全のため速度上限を設定
        velocity_cmd_z = np.clip(velocity_cmd_z, -MAX_VELOCITY, MAX_VELOCITY)

        velocity_cmd = np.array([0.0, 0.0, velocity_cmd_z])

        self.send_velocity_command(velocity_cmd)

    def send_velocity_command(self, velocity_cmd):
        # 速度コマンドを生成
        twist_cmd = TwistStamped()
        twist_cmd.header = Header()
        twist_cmd.header.stamp = self.get_clock().now().to_msg()
        twist_cmd.header.frame_id = 'tool0'
        
        # 速度ベクトルを設定
        twist_cmd.twist.linear.x = float(velocity_cmd[0])
        twist_cmd.twist.linear.y = float(velocity_cmd[1])
        twist_cmd.twist.linear.z = float(velocity_cmd[2])
        
        # 回転速度はゼロに設定
        twist_cmd.twist.angular.x = 0.0
        twist_cmd.twist.angular.y = 0.0
        twist_cmd.twist.angular.z = 0.0
        
        # 速度コマンドを発行
        self.twist_pub.publish(twist_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ForceFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

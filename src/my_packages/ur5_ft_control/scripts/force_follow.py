#!/usr/bin/env python3

"""
TFセンサーの力ベクトル方向にツールを追従する。
つまり、先端を握って力を掛けると動く。
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header
import numpy as np

FORCE_THRESHOLD = 10.0  # N, 閾値
VELOCITY_SCALE = 0.1   # スケーリング係数（力を速度に変換する際の係数）

class ForceFollowNode(Node):
    def __init__(self):
        super().__init__('force_follow_node')
        self.subscription = self.create_subscription(
            WrenchStamped,
            '/force_torque_sensor_broadcaster/wrench',
            self.wrench_callback,
            10)
        self.twist_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.initial_force = None  # 初期値を保存
        self.get_logger().info('ForceFollowNode started with velocity control.')

    def wrench_callback(self, msg):
        fx = msg.wrench.force.x
        fy = msg.wrench.force.y
        fz = msg.wrench.force.z

        current_force = np.array([fx, fy, fz])
        if self.initial_force is None:
            self.initial_force = current_force
            self.get_logger().info('Initial force set.')
            # 初期値設定時は動作しない
            return

        force_vec = current_force - self.initial_force  # 初期値との差分
        norm = np.linalg.norm(force_vec)
        self.get_logger().info(f'Force norm: {norm}')
        if norm > FORCE_THRESHOLD:
            direction = force_vec / norm
            self.send_velocity_command(direction, norm)
            # self.get_logger().info(f'Force detected: {force_vec}, sending velocity in direction {direction}')
        else:
            # 力が閾値以下のときは停止コマンドを送信
            self.send_velocity_command(np.array([0.0, 0.0, 0.0]), 0.0)

    def send_velocity_command(self, direction, magnitude):
        # 速度コマンドを生成
        twist_cmd = TwistStamped()
        twist_cmd.header = Header()
        twist_cmd.header.stamp = self.get_clock().now().to_msg()
        twist_cmd.header.frame_id = 'tool0'
        
        # 力の大きさに比例した速度を設定（閾値を超えた分）
        velocity_magnitude = min(magnitude - FORCE_THRESHOLD, 10.0) * VELOCITY_SCALE
        
        # 速度ベクトルを設定
        twist_cmd.twist.linear.x = direction[0] * velocity_magnitude
        twist_cmd.twist.linear.y = direction[1] * velocity_magnitude
        twist_cmd.twist.linear.z = direction[2] * velocity_magnitude
        
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

#!/usr/bin/env python3
"""Pose Visualizer - 在 RViz 中可视化机械臂预设位姿。"""

import math
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


NAMED_POSES = {
    'home': {
        'left': [0.0, 0.0, 0.0, -0.07435, 0.0, 0.0],
        'right': [0.0, 0.0, 0.0, -0.07435, 0.0, 0.0],
    },
}

ALL_JOINTS = [
    'front_left_wheel', 'front_right_wheel',
    'rear_left_wheel', 'rear_right_wheel',
    'left_joint1', 'left_joint2', 'left_joint3',
    'left_joint4', 'left_joint5', 'left_joint6',
    'left_jointGripper',
    'right_joint1', 'right_joint2', 'right_joint3',
    'right_joint4', 'right_joint5', 'right_joint6',
    'right_jointGripper',
    'pan_tilt_yaw_joint', 'pan_tilt_pitch_joint',
]

JOINT_INDEX = {name: index for index, name in enumerate(ALL_JOINTS)}


class PoseVisualizer(Node):
    def __init__(self):
        super().__init__('pose_visualizer')

        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.positions = [0.0] * len(ALL_JOINTS)

        self.declare_parameter('pose_name', 'home')
        self.declare_parameter('publish_rate', 50.0)
        self.pose_name = self.get_parameter('pose_name').value
        publish_rate = self.get_parameter('publish_rate').value

        self._set_arm_pose(self.pose_name)
        self.create_timer(1.0 / float(publish_rate), self.publish_joint_states)

        self.command_thread = threading.Thread(target=self._command_loop, daemon=True)
        self.command_thread.start()

    def _set_arm_pose(self, pose_name: str) -> bool:
        if pose_name not in NAMED_POSES:
            return False

        pose = NAMED_POSES[pose_name]
        for index, value in enumerate(pose['left']):
            self.positions[JOINT_INDEX[f'left_joint{index + 1}']] = value
        for index, value in enumerate(pose['right']):
            self.positions[JOINT_INDEX[f'right_joint{index + 1}']] = value
        return True

    def set_pose(self, pose_name):
        if self._set_arm_pose(pose_name):
            print(f'切换到位姿: {pose_name}')
            self.print_pose_info(pose_name)
            return True
        return False

    def set_head(self, pan: float, tilt: float):
        self.positions[JOINT_INDEX['pan_tilt_yaw_joint']] = pan
        self.positions[JOINT_INDEX['pan_tilt_pitch_joint']] = tilt

    def set_gripper(self, side: str, position: float):
        joint_name = f'{side}_jointGripper'
        if joint_name in JOINT_INDEX:
            self.positions[JOINT_INDEX[joint_name]] = position

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ALL_JOINTS
        msg.position = self.positions
        msg.velocity = [0.0] * len(ALL_JOINTS)
        msg.effort = [0.0] * len(ALL_JOINTS)
        self.publisher.publish(msg)

    def print_pose_info(self, pose_name):
        pose = NAMED_POSES.get(pose_name)
        if pose is None:
            return
        print(f"\n{'=' * 50}")
        print(f"位姿: {pose_name}")
        print(f"{'=' * 50}")
        print(f"{'关节':<10} {'弧度':>10} {'角度':>10}")
        print(f"{'-' * 32}")

        for name, rad in zip(
                ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
                pose['left']):
            print(f'{name:<10} {rad:>10.4f} {math.degrees(rad):>9.1f}°')

    def _command_loop(self):
        print('\n机械臂位姿可视化工具')
        print('可用命令: home | head <pan> <tilt> | grip <open/close> | help | q')
        self.print_pose_info(self.pose_name)

        while rclpy.ok():
            try:
                command = input('输入命令 > ').strip().lower()
            except EOFError:
                return
            except KeyboardInterrupt:
                return

            if not command:
                continue
            if command in {'q', 'quit'}:
                print('退出')
                rclpy.shutdown()
                return
            if command in NAMED_POSES:
                self.set_pose(command)
                continue
            if command.startswith('head '):
                try:
                    _, pan, tilt = command.split(maxsplit=2)
                    self.set_head(float(pan), float(tilt))
                    print(f'云台: pan={float(pan):.2f}, tilt={float(tilt):.2f}')
                except ValueError:
                    print('用法: head <pan> <tilt>')
                continue
            if command.startswith('grip '):
                action = command.split(maxsplit=1)[1]
                position = -1.0 if action == 'open' else 0.0
                self.set_gripper('left', position)
                self.set_gripper('right', position)
                print(f'夹爪: {action}')
                continue
            if command == 'help':
                print('\n可用命令:')
                print('  home - 切换到 home 位姿')
                print('  head <pan> <tilt> - 设置云台(弧度)')
                print('  grip open/close - 控制夹爪')
                print('  q - 退出\n')
                continue
            print(f"未知命令: {command}, 输入 'help' 查看帮助")


def main(args=None):
    rclpy.init(args=args)
    node = PoseVisualizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

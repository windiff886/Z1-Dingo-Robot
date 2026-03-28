#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Keyboard Teleop - 键盘遥控 Uni-Dingo 机器人

按键映射:
    底盘控制:
        w/s     - 前进/后退 (Shift 加速)
        a/d     - 左移/右移 (Shift 加速)
        q/e     - 左转/右转 (Shift 加速)
        Space   - 急停

    机械臂控制:
        1       - 双臂 Home
        2       - 左臂 Home
        3       - 右臂 Home

    末端位姿控制 (当前选中臂):
        t/g     - 末端上升/下降 (Shift 加速)
        f/h     - 末端左移/右移
        r/y     - 末端前进/后退 (Shift 加速)
        Tab     - 切换控制左/右臂

    夹爪控制:
        o/p     - 左臂夹爪 开/关
        [/]     - 右臂夹爪 开/关

    云台控制:
        i/k     - 上仰/下俯 (Shift 加速)
        j/l     - 左转/右转 (Shift 加速)
        H       - 云台归零

    系统:
        ESC/Ctrl+C  - 退出

Usage:
    ros2 run whole_body_example keyboard_teleop
    ros2 run whole_body_example keyboard_teleop --ros-args -p mode:=simulation
"""

import sys
import select
import termios
import tty
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

from uni_dingo_api.api.robot_api import RobotAPI


MOVE_BINDINGS = {
    'w': (1, 0, 0),
    's': (-1, 0, 0),
    'a': (0, 1, 0),
    'd': (0, -1, 0),
    'q': (0, 0, 1),
    'e': (0, 0, -1),
    'W': (2, 0, 0),
    'S': (-2, 0, 0),
    'A': (0, 2, 0),
    'D': (0, -2, 0),
    'Q': (0, 0, 2),
    'E': (0, 0, -2),
}

HEAD_BINDINGS = {
    'i': (0, 0.1),
    'k': (0, -0.1),
    'j': (0.1, 0),
    'l': (-0.1, 0),
    'I': (0, 0.2),
    'K': (0, -0.2),
    'J': (0.2, 0),
    'L': (-0.2, 0),
}

EE_POSE_BINDINGS = {
    't': (0, 0, 0.02),
    'g': (0, 0, -0.02),
    'f': (0, 0.02, 0),
    'h': (0, -0.02, 0),
    'r': (0.02, 0, 0),
    'y': (-0.02, 0, 0),
    'T': (0, 0, 0.05),
    'G': (0, 0, -0.05),
    'F': (0, 0.05, 0),
    'R': (0.05, 0, 0),
    'Y': (-0.05, 0, 0),
}


HELP_MSG = """
============================================
    Uni-Dingo 键盘遥控
============================================

底盘控制:
    w/s     前进/后退      (Shift 加速)
    a/d     左移/右移      (Shift 加速)
    q/e     左转/右转      (Shift 加速)
    空格    急停

机械臂控制:
    1       双臂 Home
    2       左臂 Home
    3       右臂 Home

末端位姿控制 (当前选中臂):
    t/g     末端上升/下降  (Shift 加速)
    f/h     末端左移/右移
    r/y     末端前进/后退  (Shift 加速)
    Tab     切换控制左/右臂

夹爪控制:
    o/p     左臂夹爪 开/关
    [/]     右臂夹爪 开/关

云台控制:
    i/k     上仰/下俯      (Shift 加速)
    j/l     左转/右转      (Shift 加速)
    H       云台归零

系统:
    ESC     退出

============================================
"""


class KeyboardTeleop:
    def __init__(self, node: Node):
        self._node = node
        self.linear_speed = 0.3
        self.angular_speed = 0.5

        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0

        self.head_pan = 0.0
        self.head_tilt = 0.0

        self.running = True
        self.arm_busy = False
        self.active_arm = 'left'

        self.settings = termios.tcgetattr(sys.stdin)

        node.declare_parameter('mode', '')
        mode = node.get_parameter('mode').get_parameter_value().string_value
        mode = mode if mode else None

        node.get_logger().info('Initializing RobotAPI...')
        self.robot = RobotAPI(mode=mode, node=node)

        self.pub_thread = threading.Thread(target=self._publish_loop, daemon=True)
        self.pub_thread.start()

    def _get_key(self, timeout=0.1):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def _publish_loop(self):
        while self.running and rclpy.ok():
            if self.robot.base:
                self.robot.base.move(
                    linear_x=self.vx,
                    linear_y=self.vy,
                    angular_z=self.vth
                )
            threading.Event().wait(0.05)

    def _move_arm(self, arm, pose_name):
        if arm is None:
            self._node.get_logger().warn('Arm not enabled')
            return

        if self.arm_busy:
            self._node.get_logger().warn('Arm is busy...')
            return

        self.arm_busy = True
        self._node.get_logger().info(f'Moving arm to {pose_name}...')

        def do_move():
            try:
                future = arm.move_to_named(pose_name)
                future.wait(timeout=30.0)
                if future.is_success():
                    self._node.get_logger().info(f'Arm reached {pose_name}')
                else:
                    self._node.get_logger().warn(f'Arm motion failed: {future.get_error()}')
            except Exception as e:
                self._node.get_logger().error(f'Arm error: {e}')
            finally:
                self.arm_busy = False

        threading.Thread(target=do_move, daemon=True).start()

    def _control_gripper(self, side, action):
        arm = self.robot.left_arm if side == 'left' else self.robot.right_arm
        if arm is None:
            self._node.get_logger().warn(f'{side} arm not enabled')
            return

        if action == 'open':
            self._node.get_logger().info(f'{side} gripper open')
            arm.gripper_open()
        else:
            self._node.get_logger().info(f'{side} gripper close')
            arm.gripper_close()

    def _move_ee_relative(self, dx, dy, dz):
        if self.active_arm == 'left':
            arm = self.robot.left_arm
        else:
            arm = self.robot.right_arm

        if arm is None:
            self._node.get_logger().warn(f'{self.active_arm} arm not enabled')
            return

        if self.arm_busy:
            self._node.get_logger().warn('Arm is busy...')
            return

        current_pose = arm.get_end_effector_pose()
        if current_pose is None:
            self._node.get_logger().warn('Cannot get current end-effector pose')
            return

        target_pose = Pose()
        target_pose.position.x = current_pose.position.x + dx
        target_pose.position.y = current_pose.position.y + dy
        target_pose.position.z = current_pose.position.z + dz
        target_pose.orientation = current_pose.orientation

        self.arm_busy = True
        self._node.get_logger().info(
            f'{self.active_arm} arm EE move: dx={dx:.3f}, dy={dy:.3f}, dz={dz:.3f}')

        def do_move():
            try:
                future = arm.move_to_pose(target_pose)
                future.wait(timeout=10.0)
                if future.is_success():
                    self._node.get_logger().info('EE move complete')
                else:
                    self._node.get_logger().warn(f'EE move failed: {future.get_error()}')
            except Exception as e:
                self._node.get_logger().error(f'EE move error: {e}')
            finally:
                self.arm_busy = False

        threading.Thread(target=do_move, daemon=True).start()

    def _switch_active_arm(self):
        if self.active_arm == 'left':
            self.active_arm = 'right'
            self._node.get_logger().info('Switched to right arm')
        else:
            self.active_arm = 'left'
            self._node.get_logger().info('Switched to left arm')

    def _move_head(self, delta_pan, delta_tilt):
        if self.robot.head is None:
            return

        self.head_pan += delta_pan
        self.head_tilt += delta_tilt
        self.head_pan = max(-1.57, min(1.57, self.head_pan))
        self.head_tilt = max(-0.5, min(0.5, self.head_tilt))
        self.robot.head.set_angles(self.head_pan, self.head_tilt)

    def _print_status(self):
        status = f'\r速度: vx={self.vx:.2f} vy={self.vy:.2f} vth={self.vth:.2f}'
        status += f' | 控制: {self.active_arm}臂'
        if self.robot.head:
            status += f' | 云台: pan={self.head_pan:.2f} tilt={self.head_tilt:.2f}'
        status += '    '
        sys.stdout.write(status)
        sys.stdout.flush()

    def run(self):
        print(HELP_MSG)
        print('等待键盘输入...\n')

        try:
            while self.running and rclpy.ok():
                key = self._get_key()

                if key == '':
                    self.vx = 0
                    self.vy = 0
                    self.vth = 0

                elif key == '\x1b':  # ESC
                    self._node.get_logger().info('退出...')
                    break

                elif key == '\x03':  # Ctrl+C
                    break

                elif key == ' ':
                    self.vx = 0
                    self.vy = 0
                    self.vth = 0
                    self.robot.stop_all()
                    self._node.get_logger().warn('急停!')

                elif key == '\t':
                    self._switch_active_arm()

                elif key in MOVE_BINDINGS:
                    x, y, th = MOVE_BINDINGS[key]
                    self.vx = x * self.linear_speed
                    self.vy = y * self.linear_speed
                    self.vth = th * self.angular_speed

                elif key in HEAD_BINDINGS:
                    delta_pan, delta_tilt = HEAD_BINDINGS[key]
                    self._move_head(delta_pan, delta_tilt)

                elif key == 'H':
                    self.head_pan = 0
                    self.head_tilt = 0
                    if self.robot.head:
                        self.robot.head.home()
                    self._node.get_logger().info('云台归零')

                elif key in EE_POSE_BINDINGS:
                    dx, dy, dz = EE_POSE_BINDINGS[key]
                    self._move_ee_relative(dx, dy, dz)

                elif key == '1':
                    self._move_arm(self.robot.arms, 'home')

                elif key == '2':
                    self._move_arm(self.robot.left_arm, 'home')

                elif key == '3':
                    self._move_arm(self.robot.right_arm, 'home')

                elif key == 'o':
                    self._control_gripper('left', 'open')

                elif key == 'p':
                    self._control_gripper('left', 'close')

                elif key == '[':
                    self._control_gripper('right', 'open')

                elif key == ']':
                    self._control_gripper('right', 'close')

                self._print_status()

        except Exception as e:
            self._node.get_logger().error(f'错误: {e}')

        finally:
            self.shutdown()

    def shutdown(self):
        self.running = False
        self.vx = 0
        self.vy = 0
        self.vth = 0

        if self.robot.base:
            self.robot.base.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        print('\n退出键盘遥控')


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('keyboard_teleop')

    try:
        teleop = KeyboardTeleop(node)
        teleop.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

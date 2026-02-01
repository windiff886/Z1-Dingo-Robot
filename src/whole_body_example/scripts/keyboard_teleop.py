#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Keyboard Teleop - 键盘遥控 Uni-Dingo 机器人

按键映射:
    底盘控制:
        w/s     - 前进/后退 (Shift 加速)
        a/d     - 左移/右移 (Shift 加速)
        q/e     - 左转/右转 (Shift 加速)
        空格    - 急停

    机械臂控制:
        1       - 双臂 Home 位姿
        2       - 左臂 Home
        3       - 右臂 Home

    末端位姿控制 (选定臂):
        t/g     - 末端上升/下降 (Shift 加速)
        f       - 末端左移 (Shift 加速)
        h       - 末端右移 (小写，无快速模式)
        r/y     - 末端前进/后退 (Shift 加速)
        Tab     - 切换控制的机械臂 (左/右)

    夹爪控制:
        o/p     - 左臂夹爪 开/关
        [/]     - 右臂夹爪 开/关

    云台控制:
        i/k     - 云台上仰/下俯 (Shift 加速)
        j/l     - 云台左转/右转 (Shift 加速)
        H       - 云台归零 (大写)

    系统:
        ESC/Ctrl+C  - 退出

Usage:
    rosrun whole_body_example keyboard_teleop.py
    rosrun whole_body_example keyboard_teleop.py _mode:=simulation
"""

import sys
import select
import termios
import tty
import threading
import rospy
from geometry_msgs.msg import Pose

from uni_dingo_api import RobotAPI


# 按键映射
MOVE_BINDINGS = {
    'w': (1, 0, 0),     # 前进
    's': (-1, 0, 0),    # 后退
    'a': (0, 1, 0),     # 左移
    'd': (0, -1, 0),    # 右移
    'q': (0, 0, 1),     # 左转
    'e': (0, 0, -1),    # 右转
    'W': (2, 0, 0),     # 快速前进
    'S': (-2, 0, 0),    # 快速后退
    'A': (0, 2, 0),     # 快速左移
    'D': (0, -2, 0),    # 快速右移
    'Q': (0, 0, 2),     # 快速左转
    'E': (0, 0, -2),    # 快速右转
}

HEAD_BINDINGS = {
    'i': (0, 0.1),      # 上仰
    'k': (0, -0.1),     # 下俯
    'j': (0.1, 0),      # 左转
    'l': (-0.1, 0),     # 右转
    'I': (0, 0.2),      # 快速上仰
    'K': (0, -0.2),     # 快速下俯
    'J': (0.2, 0),      # 快速左转
    'L': (-0.2, 0),     # 快速右转
}

# 末端位姿控制 (dx, dy, dz) 单位: 米
EE_POSE_BINDINGS = {
    't': (0, 0, 0.02),      # 上升
    'g': (0, 0, -0.02),     # 下降
    'f': (0, 0.02, 0),      # 左移
    'h': (0, -0.02, 0),     # 右移 (注意: 大写 H 用于云台归零，故无快速右移)
    'r': (0.02, 0, 0),      # 前进
    'y': (-0.02, 0, 0),     # 后退
    'T': (0, 0, 0.05),      # 快速上升
    'G': (0, 0, -0.05),     # 快速下降
    'F': (0, 0.05, 0),      # 快速左移
    'R': (0.05, 0, 0),      # 快速前进
    'Y': (-0.05, 0, 0),     # 快速后退
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
    f       末端左移       (Shift 加速)
    h       末端右移       (小写，无快速模式)
    r/y     末端前进/后退  (Shift 加速)
    Tab     切换控制左/右臂

夹爪控制:
    o/p     左臂夹爪 开/关
    [/]     右臂夹爪 开/关

云台控制:
    i/k     上仰/下俯      (Shift 加速)
    j/l     左转/右转      (Shift 加速)
    H       云台归零       (大写)

系统:
    ESC     退出

============================================
"""


class KeyboardTeleop:
    def __init__(self):
        # 速度参数
        self.linear_speed = 0.3   # m/s
        self.angular_speed = 0.5  # rad/s

        # 当前速度
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0

        # 云台角度
        self.head_pan = 0.0
        self.head_tilt = 0.0

        # 状态标志
        self.running = True
        self.arm_busy = False

        # 当前控制的机械臂 ('left' 或 'right')
        self.active_arm = 'left'

        # 终端设置
        self.settings = termios.tcgetattr(sys.stdin)

        # 初始化 RobotAPI
        mode = rospy.get_param('~mode', None)
        rospy.loginfo("Initializing RobotAPI...")
        self.robot = RobotAPI(mode=mode, init_node=False)

        # 发布线程
        self.pub_thread = threading.Thread(target=self._publish_loop, daemon=True)
        self.pub_thread.start()

    def _get_key(self, timeout=0.1):
        """非阻塞读取键盘输入"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def _publish_loop(self):
        """持续发布速度命令"""
        rate = rospy.Rate(20)
        while self.running and not rospy.is_shutdown():
            if self.robot.base:
                self.robot.base.move(
                    linear_x=self.vx,
                    linear_y=self.vy,
                    angular_z=self.vth
                )
            rate.sleep()

    def _move_arm(self, arm, pose_name):
        """异步移动机械臂"""
        if arm is None:
            rospy.logwarn("机械臂未启用")
            return

        if self.arm_busy:
            rospy.logwarn("机械臂正在运动中...")
            return

        self.arm_busy = True
        rospy.loginfo(f"移动机械臂到 {pose_name}...")

        def do_move():
            try:
                future = arm.move_to_named(pose_name)
                future.wait(timeout=30.0)
                if future.is_success():
                    rospy.loginfo(f"机械臂已到达 {pose_name}")
                else:
                    rospy.logwarn(f"机械臂运动失败: {future.get_error()}")
            except Exception as e:
                rospy.logerr(f"机械臂错误: {e}")
            finally:
                self.arm_busy = False

        thread = threading.Thread(target=do_move, daemon=True)
        thread.start()

    def _control_gripper(self, side, action):
        """控制夹爪 - 通过 ArmAPI"""
        arm = self.robot.left_arm if side == 'left' else self.robot.right_arm
        if arm is None:
            rospy.logwarn(f"{side}臂未启用")
            return

        if action == 'open':
            rospy.loginfo(f"{side}臂夹爪打开")
            arm.gripper_open()
        else:
            rospy.loginfo(f"{side}臂夹爪关闭")
            arm.gripper_close()

    def _move_ee_relative(self, dx, dy, dz):
        """相对移动末端执行器"""
        # 获取当前控制的机械臂
        if self.active_arm == 'left':
            arm = self.robot.left_arm
            arm_name = "左臂"
        else:
            arm = self.robot.right_arm
            arm_name = "右臂"

        if arm is None:
            rospy.logwarn(f"{arm_name}未启用")
            return

        if self.arm_busy:
            rospy.logwarn("机械臂正在运动中...")
            return

        # 获取当前末端位姿
        current_pose = arm.get_end_effector_pose()
        if current_pose is None:
            rospy.logwarn("无法获取当前末端位姿")
            return

        # 计算目标位姿
        target_pose = Pose()
        target_pose.position.x = current_pose.position.x + dx
        target_pose.position.y = current_pose.position.y + dy
        target_pose.position.z = current_pose.position.z + dz
        target_pose.orientation = current_pose.orientation  # 保持姿态不变

        self.arm_busy = True
        rospy.loginfo(f"{arm_name}末端移动: dx={dx:.3f}, dy={dy:.3f}, dz={dz:.3f}")

        def do_move():
            try:
                future = arm.move_to_pose(target_pose)
                future.wait(timeout=10.0)
                if future.is_success():
                    rospy.loginfo(f"{arm_name}末端移动完成")
                else:
                    rospy.logwarn(f"{arm_name}末端移动失败: {future.get_error()}")
            except Exception as e:
                rospy.logerr(f"末端移动错误: {e}")
            finally:
                self.arm_busy = False

        thread = threading.Thread(target=do_move, daemon=True)
        thread.start()

    def _switch_active_arm(self):
        """切换当前控制的机械臂"""
        if self.active_arm == 'left':
            self.active_arm = 'right'
            rospy.loginfo("切换到右臂控制")
        else:
            self.active_arm = 'left'
            rospy.loginfo("切换到左臂控制")

    def _move_head(self, delta_pan, delta_tilt):
        """移动云台"""
        if self.robot.head is None:
            return

        self.head_pan += delta_pan
        self.head_tilt += delta_tilt

        # 限制角度范围
        self.head_pan = max(-1.57, min(1.57, self.head_pan))
        self.head_tilt = max(-0.5, min(0.5, self.head_tilt))

        self.robot.head.set_angles(self.head_pan, self.head_tilt)

    def _print_status(self):
        """打印当前状态"""
        status = f"\r速度: vx={self.vx:.2f} vy={self.vy:.2f} vth={self.vth:.2f}"
        status += f" | 控制: {self.active_arm}臂"
        if self.robot.head:
            status += f" | 云台: pan={self.head_pan:.2f} tilt={self.head_tilt:.2f}"
        status += "    "  # 清除残留字符
        sys.stdout.write(status)
        sys.stdout.flush()

    def run(self):
        """主循环"""
        print(HELP_MSG)
        print("等待键盘输入...\n")

        try:
            while self.running and not rospy.is_shutdown():
                key = self._get_key()

                if key == '':
                    # 无输入,速度为0
                    self.vx = 0
                    self.vy = 0
                    self.vth = 0

                elif key == '\x1b':  # ESC
                    rospy.loginfo("退出...")
                    break

                elif key == '\x03':  # Ctrl+C
                    break

                elif key == ' ':  # 空格 - 急停
                    self.vx = 0
                    self.vy = 0
                    self.vth = 0
                    self.robot.stop_all()
                    rospy.logwarn("急停!")

                elif key == '\t':  # Tab - 切换机械臂
                    self._switch_active_arm()

                elif key in MOVE_BINDINGS:
                    x, y, th = MOVE_BINDINGS[key]
                    self.vx = x * self.linear_speed
                    self.vy = y * self.linear_speed
                    self.vth = th * self.angular_speed

                elif key in HEAD_BINDINGS:
                    delta_pan, delta_tilt = HEAD_BINDINGS[key]
                    self._move_head(delta_pan, delta_tilt)

                elif key == 'H':  # 大写H - 云台归零
                    self.head_pan = 0
                    self.head_tilt = 0
                    if self.robot.head:
                        self.robot.head.home()
                    rospy.loginfo("云台归零")

                elif key in EE_POSE_BINDINGS:
                    dx, dy, dz = EE_POSE_BINDINGS[key]
                    self._move_ee_relative(dx, dy, dz)

                elif key == '1':  # 双臂 Home
                    self._move_arm(self.robot.arms, 'home')

                elif key == '2':  # 左臂 Home
                    self._move_arm(self.robot.left_arm, 'home')

                elif key == '3':  # 右臂 Home
                    self._move_arm(self.robot.right_arm, 'home')

                elif key == 'o':  # 左臂夹爪开
                    self._control_gripper('left', 'open')

                elif key == 'p':  # 左臂夹爪关
                    self._control_gripper('left', 'close')

                elif key == '[':  # 右臂夹爪开
                    self._control_gripper('right', 'open')

                elif key == ']':  # 右臂夹爪关
                    self._control_gripper('right', 'close')

                self._print_status()

        except Exception as e:
            rospy.logerr(f"错误: {e}")

        finally:
            self.shutdown()

    def shutdown(self):
        """清理并退出"""
        self.running = False
        self.vx = 0
        self.vy = 0
        self.vth = 0

        if self.robot.base:
            self.robot.base.stop()

        # 恢复终端设置
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        print("\n退出键盘遥控")


def main():
    rospy.init_node('keyboard_teleop', anonymous=True)

    try:
        teleop = KeyboardTeleop()
        teleop.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()

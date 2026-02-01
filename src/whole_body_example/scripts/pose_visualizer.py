#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Pose Visualizer - 在 RViz 中可视化机械臂预设位姿

用法:
    roslaunch whole_body_example pose_visualizer.launch

    然后在终端中输入位姿名称:
    - home
    - 或输入关节角度
"""

import rospy
from sensor_msgs.msg import JointState
import math

# 预设位姿 (弧度)
NAMED_POSES = {
    'home': {
        'left': [0.0, 0.0, 0.0, -0.07435, 0.0, 0.0],
        'right': [0.0, 0.0, 0.0, -0.07435, 0.0, 0.0]
    }
}

# 所有关节名称
ALL_JOINTS = [
    # 底盘轮子
    'front_left_wheel', 'front_right_wheel',
    'rear_left_wheel', 'rear_right_wheel',
    # 左臂
    'left_joint1', 'left_joint2', 'left_joint3',
    'left_joint4', 'left_joint5', 'left_joint6',
    'left_jointGripper',
    # 右臂
    'right_joint1', 'right_joint2', 'right_joint3',
    'right_joint4', 'right_joint5', 'right_joint6',
    'right_jointGripper',
    # 云台
    'pan_tilt_yaw_joint', 'pan_tilt_pitch_joint',
]

# 关节索引映射
JOINT_INDEX = {name: i for i, name in enumerate(ALL_JOINTS)}


class PoseVisualizer:
    def __init__(self):
        self.pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

        # 所有关节位置 (初始化为 0)
        self.positions = [0.0] * len(ALL_JOINTS)

        # 设置初始机械臂位姿为 home
        self._set_arm_pose('home')

        # 发布频率
        self.rate = rospy.Rate(50)

    def _set_arm_pose(self, pose_name):
        """设置机械臂位姿"""
        if pose_name not in NAMED_POSES:
            return False

        pose = NAMED_POSES[pose_name]

        # 左臂
        for i, val in enumerate(pose['left']):
            joint_name = f'left_joint{i+1}'
            self.positions[JOINT_INDEX[joint_name]] = val

        # 右臂
        for i, val in enumerate(pose['right']):
            joint_name = f'right_joint{i+1}'
            self.positions[JOINT_INDEX[joint_name]] = val

        return True

    def set_pose(self, pose_name):
        """设置预设位姿"""
        if self._set_arm_pose(pose_name):
            rospy.loginfo(f"切换到位姿: {pose_name}")
            self.print_pose_info(pose_name)
            return True
        return False

    def set_head(self, pan, tilt):
        """设置云台角度"""
        self.positions[JOINT_INDEX['pan_tilt_yaw_joint']] = pan
        self.positions[JOINT_INDEX['pan_tilt_pitch_joint']] = tilt

    def set_gripper(self, side, position):
        """设置夹爪位置"""
        joint_name = f'{side}_jointGripper'
        if joint_name in JOINT_INDEX:
            self.positions[JOINT_INDEX[joint_name]] = position

    def print_pose_info(self, pose_name):
        """打印位姿信息"""
        if pose_name not in NAMED_POSES:
            return

        pose = NAMED_POSES[pose_name]
        print(f"\n{'='*50}")
        print(f"位姿: {pose_name}")
        print(f"{'='*50}")
        print(f"{'关节':<10} {'弧度':>10} {'角度':>10}")
        print(f"{'-'*32}")

        for i, (name, rad) in enumerate(zip(
                ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
                pose['left'])):
            deg = math.degrees(rad)
            print(f"{name:<10} {rad:>10.4f} {deg:>10.1f}°")
        print()

    def publish_joint_states(self):
        """发布关节状态"""
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = ALL_JOINTS
        msg.position = self.positions
        msg.velocity = [0.0] * len(ALL_JOINTS)
        msg.effort = [0.0] * len(ALL_JOINTS)
        self.pub.publish(msg)

    def run(self):
        """主循环"""
        print("\n" + "="*50)
        print("  机械臂位姿可视化工具")
        print("="*50)
        print("\n可用位姿: home")
        print("\n命令:")
        print("  home              - 切换到 home 位姿")
        print("  head <pan> <tilt> - 设置云台角度(弧度)")
        print("  grip <open/close> - 控制夹爪")
        print("  q                 - 退出")
        print("="*50 + "\n")

        self.print_pose_info('home')

        import threading

        # 发布线程
        def publish_loop():
            while not rospy.is_shutdown():
                self.publish_joint_states()
                self.rate.sleep()

        pub_thread = threading.Thread(target=publish_loop, daemon=True)
        pub_thread.start()

        # 命令循环
        while not rospy.is_shutdown():
            try:
                cmd = input("输入命令 > ").strip().lower()

                if not cmd:
                    continue

                if cmd == 'q' or cmd == 'quit':
                    break

                elif cmd in NAMED_POSES:
                    self.set_pose(cmd)

                elif cmd.startswith('head '):
                    try:
                        parts = cmd.split()[1:]
                        pan = float(parts[0])
                        tilt = float(parts[1]) if len(parts) > 1 else 0.0
                        self.set_head(pan, tilt)
                        print(f"云台: pan={pan:.2f}, tilt={tilt:.2f}")
                    except Exception as e:
                        print(f"格式错误: {e}")
                        print("用法: head 0.5 -0.3")

                elif cmd.startswith('grip '):
                    action = cmd.split()[1]
                    pos = -1.0 if action == 'open' else 0.0
                    self.set_gripper('left', pos)
                    self.set_gripper('right', pos)
                    print(f"夹爪: {action}")

                elif cmd == 'help':
                    print("\n可用命令:")
                    print("  home - 切换到 home 位姿")
                    print("  head <pan> <tilt> - 设置云台(弧度)")
                    print("  grip open/close - 控制夹爪")
                    print("  q - 退出\n")

                else:
                    print(f"未知命令: {cmd}, 输入 'help' 查看帮助")

            except EOFError:
                break
            except KeyboardInterrupt:
                break

        print("\n退出")


def main():
    rospy.init_node('pose_visualizer', anonymous=True)
    visualizer = PoseVisualizer()
    visualizer.run()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Test script for Uni-Dingo API

Usage:
    rosrun uni_dingo_api test_api.py

Make sure the robot is running first:
    roslaunch uni_dingo_api bringup.launch
"""

import rospy
import sys
from geometry_msgs.msg import Pose, Point, Quaternion


def test_base_api():
    """Test base movement."""
    print("\n=== Testing BaseAPI ===")

    from uni_dingo_api import RobotAPI
    robot = RobotAPI(init_node=False)

    if robot.base is None:
        print("Base not available, skipping...")
        return

    # Get current pose
    pose = robot.base.get_pose_2d()
    if pose:
        print(f"Current pose: x={pose.x:.3f}, y={pose.y:.3f}, theta={pose.theta:.3f}")

    # Test velocity command
    print("Sending velocity command (0.2 m/s forward)...")
    robot.base.move(linear_x=0.2)
    rospy.sleep(1.0)
    robot.base.stop()
    print("Stopped")

    # Get new pose
    pose = robot.base.get_pose_2d()
    if pose:
        print(f"New pose: x={pose.x:.3f}, y={pose.y:.3f}, theta={pose.theta:.3f}")


def test_arm_api():
    """Test arm movement."""
    print("\n=== Testing ArmAPI ===")

    from uni_dingo_api import RobotAPI
    robot = RobotAPI(init_node=False)

    if robot.left_arm is None:
        print("Left arm not available, skipping...")
        return

    # Get current joint states
    joints = robot.left_arm.get_joint_states()
    if joints:
        print(f"Left arm joint positions: {[f'{p:.3f}' for p in joints['positions']]}")

    # Get end effector pose
    ee_pose = robot.left_arm.get_end_effector_pose()
    if ee_pose:
        print(f"Left EE position: x={ee_pose.position.x:.3f}, "
              f"y={ee_pose.position.y:.3f}, z={ee_pose.position.z:.3f}")

    # Move to home
    print("Moving left arm to 'home' pose...")
    future = robot.left_arm.move_to_named('home')
    future.wait()
    if future.is_success():
        print("Success!")
    else:
        print(f"Failed: {future.get_error()}")


def test_dual_arm():
    """Test dual-arm coordination."""
    print("\n=== Testing Dual-Arm ===")

    from uni_dingo_api import RobotAPI
    robot = RobotAPI(init_node=False)

    if robot.arms is None:
        print("Dual-arm not available, skipping...")
        return

    # Move both arms to home
    print("Moving both arms to 'home' pose...")
    future = robot.arms.move_to_named('home')
    future.wait()
    if future.is_success():
        print("Success!")
    else:
        print(f"Failed: {future.get_error()}")


def test_robot_state():
    """Test robot state query."""
    print("\n=== Testing Robot State ===")

    from uni_dingo_api import RobotAPI
    robot = RobotAPI(init_node=False)

    state = robot.get_state()
    print(f"Mode: {state['mode']}")
    print(f"Emergency stopped: {state['emergency_stopped']}")

    if 'base' in state:
        print(f"Base enabled: {state['base']['enabled']}")
        print(f"Base moving: {state['base']['moving']}")

    if 'left_arm' in state:
        print(f"Left arm enabled: {state['left_arm']['enabled']}")


def main():
    rospy.init_node('test_uni_dingo_api', anonymous=True)

    print("=" * 50)
    print("Uni-Dingo API Test Suite")
    print("=" * 50)

    # Wait for system to be ready
    print("\nWaiting for robot to be ready...")
    rospy.sleep(2.0)

    try:
        test_robot_state()
        test_base_api()
        test_arm_api()
        test_dual_arm()

        print("\n" + "=" * 50)
        print("All tests completed!")
        print("=" * 50)

    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"\nTest error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()

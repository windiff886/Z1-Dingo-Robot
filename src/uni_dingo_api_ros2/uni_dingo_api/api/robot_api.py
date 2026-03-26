#!/usr/bin/env python3
"""RobotAPI - Unified interface for the complete Uni-Dingo robot (ROS2)."""

import yaml
import os
from typing import Optional, Dict, Any

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from .base_api import BaseAPI
from .arm_api import ArmAPI
from .head_api import HeadAPI
from .perception_api import PerceptionAPI
from .action_future import ActionFuture


class RobotAPI:
    """
    Unified API for the complete Uni-Dingo robot.

    Usage:
        import rclpy
        rclpy.init()
        from uni_dingo_api import RobotAPI
        robot = RobotAPI()
        robot.base.move(linear_x=0.5)
        robot.left_arm.move_to_named('home').wait()
    """

    def __init__(self, mode: str = None, config_path: str = None,
                 node: Node = None):
        if node is None:
            self._node = rclpy.create_node('uni_dingo_api')
            self._owns_node = True
        else:
            self._node = node
            self._owns_node = False

        self._config = self._load_config(config_path)
        self._mode = mode or self._config.get('robot', {}).get('mode', 'simulation')
        self._node.get_logger().info(f'RobotAPI initializing in {self._mode} mode')

        subsystems = self._config.get('subsystems', {})
        motion_config = self._config.get('motion', {})
        named_poses = self._config.get('named_poses', {})

        arm_config = {
            **subsystems.get('left_arm', {}),
            **motion_config,
            'named_poses': named_poses,
        }

        self._emergency_stopped = False

        # Base
        base_config = subsystems.get('base', {})
        if base_config.get('enabled', True):
            self.base = BaseAPI(self._node, mode=self._mode, config=base_config)
        else:
            self.base = None

        # Left arm
        left_config = {**arm_config, **subsystems.get('left_arm', {})}
        if left_config.get('enabled', True):
            self.left_arm = ArmAPI(self._node, side='left', mode=self._mode, config=left_config)
        else:
            self.left_arm = None

        # Right arm
        right_config = {**arm_config, **subsystems.get('right_arm', {})}
        if right_config.get('enabled', True):
            self.right_arm = ArmAPI(self._node, side='right', mode=self._mode, config=right_config)
        else:
            self.right_arm = None

        # Dual arm interface
        if self.left_arm and self.right_arm:
            self.arms = ArmAPI(self._node, side='both', mode=self._mode, config=arm_config)
        else:
            self.arms = None

        # Head
        head_config = subsystems.get('head', {})
        if head_config.get('enabled', False):
            self.head = HeadAPI(self._node, mode=self._mode, config=head_config)
        else:
            self.head = None

        # Perception
        perception_config = subsystems.get('perception', {})
        if perception_config.get('enabled', False):
            self.perception = PerceptionAPI(self._node, mode=self._mode, config=perception_config)
        else:
            self.perception = None

        self._node.get_logger().info('RobotAPI initialization complete')

    def _load_config(self, config_path: str = None) -> dict:
        if config_path is None:
            try:
                pkg_share = get_package_share_directory('uni_dingo_api')
                config_path = os.path.join(pkg_share, 'config', 'robot_config.yaml')
            except Exception:
                config_path = os.path.join(
                    os.path.dirname(__file__), '..', '..', 'config', 'robot_config.yaml')

        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                return yaml.safe_load(f) or {}
        else:
            self._node.get_logger().warn(f'Config not found: {config_path}, using defaults')
            return {}

    def get_state(self) -> Dict[str, Any]:
        state = {
            'mode': self._mode,
            'emergency_stopped': self._emergency_stopped,
        }

        if self.base:
            pose = self.base.get_pose_2d()
            vel = self.base.get_velocity()
            state['base'] = {
                'enabled': self.base.is_enabled(),
                'moving': self.base.is_moving(),
                'pose': {'x': pose.x, 'y': pose.y, 'theta': pose.theta} if pose else None,
                'velocity': {
                    'linear_x': vel.linear.x, 'linear_y': vel.linear.y,
                    'angular_z': vel.angular.z,
                } if vel else None,
            }

        for side in ['left', 'right']:
            arm = getattr(self, f'{side}_arm', None)
            if arm:
                joints = arm.get_joint_states()
                state[f'{side}_arm'] = {
                    'enabled': arm.is_enabled(),
                    'moving': arm.is_moving(),
                    'joint_positions': joints.get('positions') if joints else None,
                }

        if self.head:
            pan, tilt = self.head.get_angles()
            state['head'] = {
                'enabled': self.head.is_enabled(),
                'pan': pan, 'tilt': tilt,
            }

        if self.perception:
            state['perception'] = {
                'enabled': self.perception.is_enabled(),
                'has_data': self.perception.has_data(),
            }

        return state

    def emergency_stop(self) -> None:
        self._emergency_stopped = True
        if self.base:
            self.base.emergency_stop()
        if self.left_arm:
            self.left_arm.stop()
            self.left_arm.set_enabled(False)
        if self.right_arm:
            self.right_arm.stop()
            self.right_arm.set_enabled(False)
        if self.head:
            self.head.stop()
            self.head.set_enabled(False)
        self._node.get_logger().warn('EMERGENCY STOP ACTIVATED')

    def recover(self) -> bool:
        if self.base:
            self.base.recover()
        if self.left_arm:
            self.left_arm.set_enabled(True)
        if self.right_arm:
            self.right_arm.set_enabled(True)
        if self.head:
            self.head.set_enabled(True)
        self._emergency_stopped = False
        self._node.get_logger().info('Emergency stop released')
        return True

    def home(self) -> ActionFuture:
        if self.arms:
            return self.arms.move_to_named('home')
        elif self.left_arm:
            return self.left_arm.move_to_named('home')
        elif self.right_arm:
            return self.right_arm.move_to_named('home')
        return ActionFuture.failure('No arms available')

    def stop_all(self) -> None:
        if self.base:
            self.base.stop()
        if self.left_arm:
            self.left_arm.stop()
        if self.right_arm:
            self.right_arm.stop()
        if self.head:
            self.head.stop()

    @property
    def mode(self) -> str:
        return self._mode

    @property
    def is_simulation(self) -> bool:
        return self._mode == 'simulation'

    @property
    def node(self) -> Node:
        return self._node

    def spin(self):
        """Spin the internal node (blocking)."""
        rclpy.spin(self._node)

    def destroy(self):
        """Clean up resources."""
        if self._owns_node:
            self._node.destroy_node()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RobotAPI - Unified interface for the complete Uni-Dingo robot
"""

import rospy
import yaml
import os
from typing import Optional, Dict, Any

from .base_api import BaseAPI
from .arm_api import ArmAPI
from .action_future import ActionFuture

# Import backends
from ..backends.simulation.gazebo_base import GazeboBaseBackend
from ..backends.simulation.moveit_arm import MoveItArmBackend


class RobotAPI:
    """
    Unified API for the complete Uni-Dingo robot.

    Aggregates all subsystems (base, arms, head, perception) and provides
    a single entry point for robot control.

    Usage:
        from uni_dingo_api import RobotAPI

        robot = RobotAPI()  # Auto-detect mode from config

        # Or specify mode explicitly
        robot = RobotAPI(mode='simulation')

        # Access subsystems
        robot.base.move(linear_x=0.5)
        robot.left_arm.move_to_named('home').wait()
        robot.right_arm.move_to_pose(target_pose).wait()

        # Dual-arm operations
        robot.arms.move_both(left_pose, right_pose).wait()

        # Robot state
        state = robot.get_state()
    """

    def __init__(self, mode: str = None, config_path: str = None,
                 init_node: bool = True):
        """
        Initialize RobotAPI.

        Args:
            mode: 'simulation' or 'real'. If None, read from config.
            config_path: Path to robot_config.yaml. If None, use default.
            init_node: Whether to initialize ROS node if not already running.
        """
        # Initialize ROS node if needed
        if init_node:
            try:
                rospy.init_node('uni_dingo_api', anonymous=True)
            except rospy.ROSException:
                pass  # Node already initialized

        # Load configuration
        self._config = self._load_config(config_path)

        # Determine mode
        self._mode = mode or self._config.get('robot', {}).get('mode', 'simulation')
        rospy.loginfo(f"RobotAPI initializing in {self._mode} mode")

        # Get subsystem configs
        subsystems = self._config.get('subsystems', {})
        motion_config = self._config.get('motion', {})
        named_poses = self._config.get('named_poses', {})

        # Merge motion config into subsystem configs
        arm_config = {
            **subsystems.get('left_arm', {}),
            **motion_config,
            'named_poses': named_poses
        }

        # Initialize subsystems based on mode and config
        self._emergency_stopped = False

        # Base
        base_config = subsystems.get('base', {})
        if base_config.get('enabled', True):
            if self._mode == 'simulation':
                self.base = GazeboBaseBackend(config=base_config)
            else:
                self.base = BaseAPI(mode='real', config=base_config)
        else:
            self.base = None
            rospy.loginfo("Base subsystem disabled")

        # Left arm
        left_config = {**arm_config, **subsystems.get('left_arm', {})}
        if left_config.get('enabled', True):
            if self._mode == 'simulation':
                self.left_arm = MoveItArmBackend(side='left', config=left_config)
            else:
                self.left_arm = ArmAPI(side='left', mode='real', config=left_config)
        else:
            self.left_arm = None
            rospy.loginfo("Left arm subsystem disabled")

        # Right arm
        right_config = {**arm_config, **subsystems.get('right_arm', {})}
        if right_config.get('enabled', True):
            if self._mode == 'simulation':
                self.right_arm = MoveItArmBackend(side='right', config=right_config)
            else:
                self.right_arm = ArmAPI(side='right', mode='real', config=right_config)
        else:
            self.right_arm = None
            rospy.loginfo("Right arm subsystem disabled")

        # Dual-arm interface
        if self.left_arm and self.right_arm:
            if self._mode == 'simulation':
                self.arms = MoveItArmBackend(side='both', config=arm_config)
            else:
                self.arms = ArmAPI(side='both', mode='real', config=arm_config)
        else:
            self.arms = None

        # Head (placeholder for Phase 2)
        self.head = None
        if subsystems.get('head', {}).get('enabled', False):
            rospy.loginfo("Head subsystem: Not implemented in Phase 1")

        # Perception (placeholder for Phase 2)
        self.perception = None
        if subsystems.get('perception', {}).get('enabled', False):
            rospy.loginfo("Perception subsystem: Not implemented in Phase 1")

        rospy.loginfo("RobotAPI initialization complete")

    def _load_config(self, config_path: str = None) -> dict:
        """Load configuration from YAML file."""
        if config_path is None:
            # Try to find config in package
            try:
                import rospkg
                rospack = rospkg.RosPack()
                pkg_path = rospack.get_path('uni_dingo_api')
                config_path = os.path.join(pkg_path, 'config', 'robot_config.yaml')
            except Exception:
                # Fallback to relative path
                config_path = os.path.join(
                    os.path.dirname(__file__), '..', '..', '..', '..',
                    'config', 'robot_config.yaml'
                )

        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                return yaml.safe_load(f)
        else:
            rospy.logwarn(f"Config not found: {config_path}, using defaults")
            return {}

    # ==================== Robot State ====================

    def get_state(self) -> Dict[str, Any]:
        """
        Get complete robot state.

        Returns:
            Dictionary with all subsystem states
        """
        state = {
            'mode': self._mode,
            'emergency_stopped': self._emergency_stopped,
            'timestamp': rospy.Time.now().to_sec()
        }

        # Base state
        if self.base:
            pose = self.base.get_pose_2d()
            vel = self.base.get_velocity()
            state['base'] = {
                'enabled': self.base.is_enabled(),
                'moving': self.base.is_moving(),
                'pose': {'x': pose.x, 'y': pose.y, 'theta': pose.theta} if pose else None,
                'velocity': {
                    'linear_x': vel.linear.x,
                    'linear_y': vel.linear.y,
                    'angular_z': vel.angular.z
                } if vel else None
            }

        # Arm states
        for side in ['left', 'right']:
            arm = getattr(self, f'{side}_arm', None)
            if arm:
                joints = arm.get_joint_states()
                ee_pose = arm.get_end_effector_pose()
                state[f'{side}_arm'] = {
                    'enabled': arm.is_enabled(),
                    'moving': arm.is_moving(),
                    'joint_positions': joints.get('positions') if joints else None,
                    'end_effector_pose': {
                        'position': {
                            'x': ee_pose.position.x,
                            'y': ee_pose.position.y,
                            'z': ee_pose.position.z
                        },
                        'orientation': {
                            'x': ee_pose.orientation.x,
                            'y': ee_pose.orientation.y,
                            'z': ee_pose.orientation.z,
                            'w': ee_pose.orientation.w
                        }
                    } if ee_pose else None
                }

        return state

    def is_ready(self) -> bool:
        """Check if all enabled subsystems are ready."""
        if self._emergency_stopped:
            return False

        if self.base and not self.base.is_enabled():
            return False

        if self.left_arm and not self.left_arm.is_enabled():
            return False

        if self.right_arm and not self.right_arm.is_enabled():
            return False

        return True

    # ==================== Emergency Stop ====================

    def emergency_stop(self) -> None:
        """
        Activate emergency stop for all subsystems.
        """
        self._emergency_stopped = True

        if self.base:
            self.base.emergency_stop()

        if self.left_arm:
            self.left_arm.stop()
            self.left_arm.set_enabled(False)

        if self.right_arm:
            self.right_arm.stop()
            self.right_arm.set_enabled(False)

        rospy.logwarn("EMERGENCY STOP ACTIVATED")

    def recover(self) -> bool:
        """
        Recover from emergency stop.

        Returns:
            True if recovery successful
        """
        if self.base:
            self.base.recover()

        if self.left_arm:
            self.left_arm.set_enabled(True)

        if self.right_arm:
            self.right_arm.set_enabled(True)

        self._emergency_stopped = False
        rospy.loginfo("Emergency stop released")
        return True

    # ==================== Whole-body Motion ====================

    def whole_body_move(self, base_goal=None, left_arm_goal=None,
                        right_arm_goal=None) -> ActionFuture:
        """
        Coordinated whole-body motion.

        Moves base first (if specified), then arms simultaneously.

        Args:
            base_goal: (x, y, theta) tuple for base
            left_arm_goal: Pose or joint list for left arm
            right_arm_goal: Pose or joint list for right arm

        Returns:
            ActionFuture
        """
        if self._emergency_stopped:
            return ActionFuture.failure("Emergency stopped")

        import threading

        future = ActionFuture()

        def execute():
            try:
                success = True

                # Move base first
                if base_goal and self.base:
                    x, y, theta = base_goal
                    base_future = self.base.move_to(x, y, theta)
                    base_future.wait()
                    if not base_future.is_success():
                        success = False

                # Move arms (could be parallelized)
                arm_futures = []

                if left_arm_goal and self.left_arm:
                    if isinstance(left_arm_goal, (list, tuple)):
                        f = self.left_arm.move_to_joint(list(left_arm_goal))
                    else:
                        f = self.left_arm.move_to_pose(left_arm_goal)
                    arm_futures.append(f)

                if right_arm_goal and self.right_arm:
                    if isinstance(right_arm_goal, (list, tuple)):
                        f = self.right_arm.move_to_joint(list(right_arm_goal))
                    else:
                        f = self.right_arm.move_to_pose(right_arm_goal)
                    arm_futures.append(f)

                # Wait for arms
                for f in arm_futures:
                    f.wait()
                    if not f.is_success():
                        success = False

                future._success = success

            except Exception as e:
                future._success = False
                future._error_msg = str(e)
            finally:
                future._done.set()

        thread = threading.Thread(target=execute, daemon=True)
        thread.start()

        return future

    # ==================== Convenience Methods ====================

    def home(self) -> ActionFuture:
        """Move all arms to home position."""
        if self.arms:
            return self.arms.move_to_named('home')
        elif self.left_arm:
            return self.left_arm.move_to_named('home')
        elif self.right_arm:
            return self.right_arm.move_to_named('home')
        else:
            return ActionFuture.failure("No arms available")

    def stop_all(self) -> None:
        """Stop all motion immediately."""
        if self.base:
            self.base.stop()
        if self.left_arm:
            self.left_arm.stop()
        if self.right_arm:
            self.right_arm.stop()

    @property
    def mode(self) -> str:
        """Get current operating mode."""
        return self._mode

    @property
    def is_simulation(self) -> bool:
        """Check if running in simulation."""
        return self._mode == 'simulation'

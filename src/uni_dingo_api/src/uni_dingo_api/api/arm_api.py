#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ArmAPI - Unified interface for robotic arm control
"""

import rospy
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from typing import Optional, List, Dict, Literal
import threading

from .action_future import ActionFuture


class ArmAPI:
    """
    Unified API for robotic arm control.

    Supports both simulation (MoveIt) and real robot (Z1 SDK).
    Can control single arm or dual arms.

    Usage:
        arm = ArmAPI(side='left', mode='simulation')
        arm.move_to_named('home').wait()
        arm.move_to_pose(target_pose).wait()
        arm.gripper_close().wait()
    """

    def __init__(self, side: Literal['left', 'right', 'both'] = 'left',
                 mode: str = 'simulation', config: dict = None):
        """
        Initialize ArmAPI.

        Args:
            side: 'left', 'right', or 'both' for dual-arm control
            mode: 'simulation' or 'real'
            config: Configuration dictionary
        """
        self._side = side
        self._mode = mode
        self._config = config or {}
        self._enabled = True

        # MoveIt interface (lazy initialization)
        self._move_group = None
        self._robot = None
        self._scene = None

        # Planning groups
        self._planning_groups = {
            'left': 'left_arm_manipulator',
            'right': 'right_arm_manipulator'
        }

        # End effector links
        self._ee_links = {
            'left': 'left_link06',
            'right': 'right_link06'
        }

        # Joint names
        self._joint_names = {
            'left': ['left_joint1', 'left_joint2', 'left_joint3',
                     'left_joint4', 'left_joint5', 'left_joint6'],
            'right': ['right_joint1', 'right_joint2', 'right_joint3',
                      'right_joint4', 'right_joint5', 'right_joint6']
        }

        # Named poses
        self._named_poses = config.get('named_poses', {}) if config else {}

        # Current joint state
        self._current_joint_state = None
        self._joint_state_lock = threading.Lock()

        # Subscribe to joint states
        self._joint_state_sub = rospy.Subscriber(
            '/joint_states', JointState, self._joint_state_callback, queue_size=10
        )

        # Gripper publishers
        self._gripper_pubs = {}
        if side in ['left', 'both']:
            self._gripper_pubs['left'] = rospy.Publisher(
                '/left_arm/left_z1_joint_traj_controller/command',
                JointTrajectory, queue_size=10
            )
        if side in ['right', 'both']:
            self._gripper_pubs['right'] = rospy.Publisher(
                '/right_arm/right_z1_joint_traj_controller/command',
                JointTrajectory, queue_size=10
            )

        # Motion parameters
        self._velocity_scaling = config.get('default_velocity_scaling', 0.3) if config else 0.3
        self._acceleration_scaling = config.get('default_acceleration_scaling', 0.3) if config else 0.3
        self._planning_time = config.get('planning_time', 5.0) if config else 5.0

        rospy.loginfo(f"ArmAPI initialized for {side} arm(s) in {mode} mode")

    def _joint_state_callback(self, msg: JointState):
        """Callback for joint state updates."""
        with self._joint_state_lock:
            self._current_joint_state = msg

    def _init_moveit(self):
        """Initialize MoveIt interfaces (lazy)."""
        if self._move_group is not None:
            return True

        try:
            import moveit_commander
            moveit_commander.roscpp_initialize([])

            self._robot = moveit_commander.RobotCommander()
            self._scene = moveit_commander.PlanningSceneInterface()

            # Create move group for the specified side
            if self._side == 'both':
                self._move_groups = {
                    'left': moveit_commander.MoveGroupCommander(
                        self._planning_groups['left']),
                    'right': moveit_commander.MoveGroupCommander(
                        self._planning_groups['right'])
                }
                self._move_group = self._move_groups['left']  # Default
            else:
                self._move_group = moveit_commander.MoveGroupCommander(
                    self._planning_groups[self._side])
                self._move_groups = {self._side: self._move_group}

            # Configure move groups
            for mg in self._move_groups.values():
                mg.set_max_velocity_scaling_factor(self._velocity_scaling)
                mg.set_max_acceleration_scaling_factor(self._acceleration_scaling)
                mg.set_planning_time(self._planning_time)
                mg.set_num_planning_attempts(10)

            rospy.loginfo("MoveIt interface initialized")
            return True

        except Exception as e:
            rospy.logerr(f"Failed to initialize MoveIt: {e}")
            return False

    # ==================== Motion Control ====================

    def move_to_pose(self, pose: Pose, frame: str = 'base_link',
                     side: str = None) -> ActionFuture:
        """
        Move end effector to target pose.

        Args:
            pose: Target pose
            frame: Reference frame
            side: Which arm ('left' or 'right'), None uses default

        Returns:
            ActionFuture
        """
        if not self._enabled:
            return ActionFuture.failure("Arm disabled")

        if not self._init_moveit():
            return ActionFuture.failure("MoveIt not available")

        side = side or self._side
        if side == 'both':
            return ActionFuture.failure("Use move_both() for dual-arm motion")

        mg = self._move_groups.get(side)
        if mg is None:
            return ActionFuture.failure(f"No move group for {side}")

        future = ActionFuture()

        def execute():
            try:
                # Create PoseStamped
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = frame
                pose_stamped.header.stamp = rospy.Time.now()
                pose_stamped.pose = pose

                mg.set_pose_target(pose_stamped)

                # Plan and execute
                success = mg.go(wait=True)
                mg.stop()
                mg.clear_pose_targets()

                future._success = success
                future._result = {'success': success}
                if not success:
                    future._error_msg = "Motion planning or execution failed"

            except Exception as e:
                future._success = False
                future._error_msg = str(e)
                rospy.logerr(f"move_to_pose error: {e}")
            finally:
                future._done.set()

        thread = threading.Thread(target=execute, daemon=True)
        thread.start()

        return future

    def move_to_joint(self, positions: List[float], side: str = None) -> ActionFuture:
        """
        Move to target joint positions.

        Args:
            positions: List of 6 joint positions (rad)
            side: Which arm ('left' or 'right')

        Returns:
            ActionFuture
        """
        if not self._enabled:
            return ActionFuture.failure("Arm disabled")

        if not self._init_moveit():
            return ActionFuture.failure("MoveIt not available")

        side = side or self._side
        if side == 'both':
            return ActionFuture.failure("Specify side for joint motion")

        if len(positions) != 6:
            return ActionFuture.failure("Expected 6 joint positions")

        mg = self._move_groups.get(side)
        if mg is None:
            return ActionFuture.failure(f"No move group for {side}")

        future = ActionFuture()

        def execute():
            try:
                mg.set_joint_value_target(positions)
                success = mg.go(wait=True)
                mg.stop()

                future._success = success
                future._result = {'success': success, 'final_joints': positions}
                if not success:
                    future._error_msg = "Joint motion failed"

            except Exception as e:
                future._success = False
                future._error_msg = str(e)
            finally:
                future._done.set()

        thread = threading.Thread(target=execute, daemon=True)
        thread.start()

        return future

    def move_to_named(self, name: str, side: str = None) -> ActionFuture:
        """
        Move to a named pose ('home', 'ready', 'stow', etc.)

        Args:
            name: Named pose
            side: Which arm

        Returns:
            ActionFuture
        """
        if not self._enabled:
            return ActionFuture.failure("Arm disabled")

        if not self._init_moveit():
            return ActionFuture.failure("MoveIt not available")

        side = side or self._side
        if side == 'both':
            # Move both arms to named pose
            return self._move_both_to_named(name)

        mg = self._move_groups.get(side)
        if mg is None:
            return ActionFuture.failure(f"No move group for {side}")

        future = ActionFuture()

        def execute():
            try:
                mg.set_named_target(name)
                success = mg.go(wait=True)
                mg.stop()

                future._success = success
                future._result = {'success': success, 'named_pose': name}
                if not success:
                    future._error_msg = f"Failed to move to {name}"

            except Exception as e:
                future._success = False
                future._error_msg = str(e)
            finally:
                future._done.set()

        thread = threading.Thread(target=execute, daemon=True)
        thread.start()

        return future

    def _move_both_to_named(self, name: str) -> ActionFuture:
        """Move both arms to a named pose."""
        future = ActionFuture()

        def execute():
            try:
                success = True
                for side_key, mg in self._move_groups.items():
                    mg.set_named_target(name)
                    if not mg.go(wait=True):
                        success = False
                    mg.stop()

                future._success = success
                future._result = {'success': success}

            except Exception as e:
                future._success = False
                future._error_msg = str(e)
            finally:
                future._done.set()

        thread = threading.Thread(target=execute, daemon=True)
        thread.start()

        return future

    def move_both(self, left_pose: Pose, right_pose: Pose,
                  frame: str = 'base_link') -> ActionFuture:
        """
        Move both arms simultaneously.

        Args:
            left_pose: Target pose for left arm
            right_pose: Target pose for right arm
            frame: Reference frame

        Returns:
            ActionFuture
        """
        if self._side != 'both':
            return ActionFuture.failure("Dual-arm mode required")

        if not self._enabled:
            return ActionFuture.failure("Arms disabled")

        if not self._init_moveit():
            return ActionFuture.failure("MoveIt not available")

        future = ActionFuture()

        def execute():
            try:
                # Set targets for both arms
                left_stamped = PoseStamped()
                left_stamped.header.frame_id = frame
                left_stamped.pose = left_pose

                right_stamped = PoseStamped()
                right_stamped.header.frame_id = frame
                right_stamped.pose = right_pose

                self._move_groups['left'].set_pose_target(left_stamped)
                self._move_groups['right'].set_pose_target(right_stamped)

                # Execute sequentially (for simplicity)
                # TODO: Implement true simultaneous execution
                left_success = self._move_groups['left'].go(wait=True)
                self._move_groups['left'].stop()

                right_success = self._move_groups['right'].go(wait=True)
                self._move_groups['right'].stop()

                self._move_groups['left'].clear_pose_targets()
                self._move_groups['right'].clear_pose_targets()

                future._success = left_success and right_success

            except Exception as e:
                future._success = False
                future._error_msg = str(e)
            finally:
                future._done.set()

        thread = threading.Thread(target=execute, daemon=True)
        thread.start()

        return future

    def stop(self, side: str = None) -> None:
        """Stop arm motion."""
        if self._move_group is None:
            return

        side = side or self._side
        if side == 'both':
            for mg in self._move_groups.values():
                mg.stop()
        else:
            mg = self._move_groups.get(side)
            if mg:
                mg.stop()

    # ==================== Gripper Control ====================

    def gripper_open(self, side: str = None, position: float = -1.57) -> ActionFuture:
        """
        Open the gripper.

        Args:
            side: Which arm
            position: Target position (default -1.57 rad = full open)

        Returns:
            ActionFuture
        """
        return self._gripper_command(side or self._side, position)

    def gripper_close(self, side: str = None, position: float = 0.0) -> ActionFuture:
        """
        Close the gripper.

        Args:
            side: Which arm
            position: Target position (default 0.0 = closed)

        Returns:
            ActionFuture
        """
        return self._gripper_command(side or self._side, position)

    def gripper_set(self, position: float, side: str = None) -> ActionFuture:
        """
        Set gripper to specific position.

        Args:
            position: Target position (rad)
            side: Which arm

        Returns:
            ActionFuture
        """
        return self._gripper_command(side or self._side, position)

    def _gripper_command(self, side: str, position: float) -> ActionFuture:
        """Send gripper command via trajectory controller."""
        if side == 'both':
            # Command both grippers
            f1 = self._gripper_command('left', position)
            f2 = self._gripper_command('right', position)
            # Return composite future
            future = ActionFuture()

            def wait_both():
                f1.wait()
                f2.wait()
                future._success = f1.is_success() and f2.is_success()
                future._done.set()

            thread = threading.Thread(target=wait_both, daemon=True)
            thread.start()
            return future

        # Single gripper command
        future = ActionFuture.success()

        try:
            # For now, gripper is controlled via trajectory controller
            # In a full implementation, this would use a dedicated gripper action
            rospy.loginfo(f"Gripper {side}: set to {position}")
            # The actual gripper control would be implemented based on hardware
        except Exception as e:
            return ActionFuture.failure(str(e))

        return future

    # ==================== State Query ====================

    def get_joint_states(self, side: str = None) -> Optional[Dict]:
        """
        Get current joint states.

        Returns:
            Dict with 'positions', 'velocities', 'efforts'
        """
        side = side or self._side
        if side == 'both':
            return {
                'left': self.get_joint_states('left'),
                'right': self.get_joint_states('right')
            }

        with self._joint_state_lock:
            if self._current_joint_state is None:
                return None

            joint_names = self._joint_names.get(side, [])
            result = {'names': [], 'positions': [], 'velocities': [], 'efforts': []}

            for name in joint_names:
                if name in self._current_joint_state.name:
                    idx = self._current_joint_state.name.index(name)
                    result['names'].append(name)
                    result['positions'].append(self._current_joint_state.position[idx])
                    if idx < len(self._current_joint_state.velocity):
                        result['velocities'].append(self._current_joint_state.velocity[idx])
                    if idx < len(self._current_joint_state.effort):
                        result['efforts'].append(self._current_joint_state.effort[idx])

            # Add original timestamp from joint state message
            result['timestamp'] = self._current_joint_state.header.stamp.to_sec()

            return result

    def get_end_effector_pose(self, side: str = None,
                               frame: str = 'base_link') -> Optional[Pose]:
        """Get current end effector pose."""
        if not self._init_moveit():
            return None

        side = side or self._side
        if side == 'both':
            return {
                'left': self.get_end_effector_pose('left', frame),
                'right': self.get_end_effector_pose('right', frame)
            }

        mg = self._move_groups.get(side)
        if mg is None:
            return None

        try:
            return mg.get_current_pose().pose
        except Exception as e:
            rospy.logwarn(f"Failed to get EE pose: {e}")
            return None

    def is_moving(self, side: str = None) -> bool:
        """Check if arm is currently moving."""
        # Simple velocity-based check
        states = self.get_joint_states(side)
        if states is None:
            return False

        if isinstance(states, dict) and 'left' in states:
            # Both arms
            return self.is_moving('left') or self.is_moving('right')

        velocities = states.get('velocities', [])
        return any(abs(v) > 0.01 for v in velocities)

    # ==================== Enable/Disable ====================

    def set_enabled(self, enabled: bool) -> None:
        """Enable or disable the arm."""
        self._enabled = enabled
        if not enabled:
            self.stop()

    def is_enabled(self) -> bool:
        """Check if arm is enabled."""
        return self._enabled

    # ==================== Configuration ====================

    def set_velocity_scaling(self, scaling: float) -> None:
        """Set velocity scaling factor (0.0 to 1.0)."""
        self._velocity_scaling = max(0.01, min(1.0, scaling))
        if self._move_group:
            for mg in self._move_groups.values():
                mg.set_max_velocity_scaling_factor(self._velocity_scaling)

    def set_acceleration_scaling(self, scaling: float) -> None:
        """Set acceleration scaling factor (0.0 to 1.0)."""
        self._acceleration_scaling = max(0.01, min(1.0, scaling))
        if self._move_group:
            for mg in self._move_groups.values():
                mg.set_max_acceleration_scaling_factor(self._acceleration_scaling)

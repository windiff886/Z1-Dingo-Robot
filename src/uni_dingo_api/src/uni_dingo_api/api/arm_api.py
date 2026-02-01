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
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from typing import Optional, List, Dict, Literal
import threading
import actionlib
import os
from datetime import datetime

from .action_future import ActionFuture

# Debug log file path (relative to project root)
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.normpath(os.path.join(_SCRIPT_DIR, '..', '..', '..', '..', '..'))
ARM_DEBUG_LOG_FILE = os.path.join(_PROJECT_ROOT, 'logs', 'arm_debug.log')


def _write_debug_log(message: str):
    """Write debug message to log file with timestamp."""
    try:
        log_dir = os.path.dirname(ARM_DEBUG_LOG_FILE)
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)

        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        with open(ARM_DEBUG_LOG_FILE, 'a') as f:
            f.write(f"[{timestamp}] {message}\n")
    except Exception as e:
        rospy.logwarn(f"Failed to write debug log: {e}")


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

        # Gripper action clients
        self._gripper_clients = {}
        if side in ['left', 'both']:
            client = actionlib.SimpleActionClient(
                '/left_arm/left_z1_gripper', GripperCommandAction
            )
            if client.wait_for_server(rospy.Duration(2.0)):
                self._gripper_clients['left'] = client
                rospy.loginfo("  Left gripper action server connected")
            else:
                rospy.logwarn("  Left gripper action server not available")
        if side in ['right', 'both']:
            client = actionlib.SimpleActionClient(
                '/right_arm/right_z1_gripper', GripperCommandAction
            )
            if client.wait_for_server(rospy.Duration(2.0)):
                self._gripper_clients['right'] = client
                rospy.loginfo("  Right gripper action server connected")
            else:
                rospy.logwarn("  Right gripper action server not available")

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
            for side_key, mg in self._move_groups.items():
                mg.set_max_velocity_scaling_factor(self._velocity_scaling)
                mg.set_max_acceleration_scaling_factor(self._acceleration_scaling)
                mg.set_planning_time(self._planning_time)
                mg.set_num_planning_attempts(10)
                # Set end effector link and reference frame
                mg.set_end_effector_link(self._ee_links[side_key])
                mg.set_pose_reference_frame('base_link')
                rospy.loginfo(f"MoveGroup '{side_key}': ee_link={self._ee_links[side_key]}, ref_frame=base_link")

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
                _write_debug_log(f"========== move_to_pose [{side}] ==========")

                # Debug: current state
                current_joints = mg.get_current_joint_values()
                _write_debug_log(f"[{side}] Current joints: {[f'{j:.4f}' for j in current_joints]}")

                current_pose = mg.get_current_pose()
                _write_debug_log(f"[{side}] Current EE pose (frame={current_pose.header.frame_id}):")
                _write_debug_log(f"  position: x={current_pose.pose.position.x:.4f}, y={current_pose.pose.position.y:.4f}, z={current_pose.pose.position.z:.4f}")

                # Create PoseStamped
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = frame
                pose_stamped.header.stamp = rospy.Time.now()
                pose_stamped.pose = pose

                # Debug: target pose
                _write_debug_log(f"[{side}] Target pose in '{frame}':")
                _write_debug_log(f"  position: x={pose.position.x:.4f}, y={pose.position.y:.4f}, z={pose.position.z:.4f}")
                _write_debug_log(f"  orientation: x={pose.orientation.x:.4f}, y={pose.orientation.y:.4f}, z={pose.orientation.z:.4f}, w={pose.orientation.w:.4f}")

                # Debug: move group settings
                _write_debug_log(f"[{side}] MoveGroup settings:")
                _write_debug_log(f"  planning_frame: {mg.get_planning_frame()}")
                _write_debug_log(f"  end_effector_link: {mg.get_end_effector_link()}")
                _write_debug_log(f"  pose_reference_frame: {mg.get_pose_reference_frame()}")

                mg.set_pose_target(pose_stamped)

                # Plan first to check if planning succeeds
                plan = mg.plan()
                # plan() returns (success, trajectory, planning_time, error_code) in newer MoveIt
                # or just trajectory in older versions
                if isinstance(plan, tuple):
                    plan_success = plan[0]
                    trajectory = plan[1]
                else:
                    trajectory = plan
                    plan_success = trajectory is not None and len(trajectory.joint_trajectory.points) > 0

                if not plan_success:
                    _write_debug_log(f"[{side}] Motion planning FAILED - no valid trajectory found")
                    future._success = False
                    future._error_msg = "Motion planning failed - no valid IK solution or path"
                    mg.clear_pose_targets()
                    return

                # Debug: trajectory info
                traj = trajectory.joint_trajectory
                _write_debug_log(f"[{side}] Planning succeeded:")
                _write_debug_log(f"  waypoints: {len(traj.points)}")
                _write_debug_log(f"  joints: {traj.joint_names}")
                if len(traj.points) > 0:
                    _write_debug_log(f"  start time: {traj.points[0].time_from_start.to_sec():.3f}s")
                    _write_debug_log(f"  end time: {traj.points[-1].time_from_start.to_sec():.3f}s")
                    _write_debug_log(f"  start pos: {[f'{p:.4f}' for p in traj.points[0].positions]}")
                    _write_debug_log(f"  end pos: {[f'{p:.4f}' for p in traj.points[-1].positions]}")

                # Execute the planned trajectory
                _write_debug_log(f"[{side}] Executing trajectory...")
                exec_success = mg.execute(trajectory, wait=True)
                mg.stop()
                mg.clear_pose_targets()

                # Debug: final state
                final_joints = mg.get_current_joint_values()
                _write_debug_log(f"[{side}] Final joints: {[f'{j:.4f}' for j in final_joints]}")

                if exec_success:
                    _write_debug_log(f"[{side}] Execution succeeded")
                else:
                    _write_debug_log(f"[{side}] Execution FAILED (controller error)")
                    # Calculate error
                    if len(traj.points) > 0:
                        target_joints = traj.points[-1].positions
                        errors = [abs(f - t) for f, t in zip(final_joints, target_joints)]
                        _write_debug_log(f"[{side}] Joint errors: {[f'{e:.4f}' for e in errors]}")
                        _write_debug_log(f"[{side}] Max error: {max(errors):.4f} rad ({max(errors)*57.3:.2f} deg)")

                future._success = exec_success
                future._result = {'success': exec_success}
                if not exec_success:
                    future._error_msg = "Trajectory execution failed (controller)"

            except Exception as e:
                future._success = False
                future._error_msg = str(e)
                _write_debug_log(f"move_to_pose error: {e}")
                import traceback
                _write_debug_log(traceback.format_exc())
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
        """Move both arms to a named pose (parallel execution)."""
        future = ActionFuture()

        def execute():
            try:
                results = {}
                threads = []

                def move_single_arm(side_key, mg):
                    """Move a single arm and store result."""
                    mg.set_named_target(name)
                    results[side_key] = mg.go(wait=True)
                    mg.stop()

                # Start both arms in parallel
                for side_key, mg in self._move_groups.items():
                    t = threading.Thread(target=move_single_arm, args=(side_key, mg))
                    threads.append(t)
                    t.start()

                # Wait for both to complete
                for t in threads:
                    t.join()

                success = all(results.values())
                future._success = success
                future._result = {'success': success, 'details': results}

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
        Move both arms simultaneously (parallel execution).

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

                # Execute in parallel
                results = {}

                def execute_arm(side_key):
                    """Execute motion for a single arm."""
                    mg = self._move_groups[side_key]
                    results[side_key] = mg.go(wait=True)
                    mg.stop()
                    mg.clear_pose_targets()

                threads = []
                for side_key in ['left', 'right']:
                    t = threading.Thread(target=execute_arm, args=(side_key,))
                    threads.append(t)
                    t.start()

                # Wait for both to complete
                for t in threads:
                    t.join()

                future._success = results.get('left', False) and results.get('right', False)
                future._result = {'success': future._success, 'details': results}

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

    def _gripper_command(self, side: str, position: float, max_effort: float = 10.0) -> ActionFuture:
        """Send gripper command via GripperCommandAction."""
        if side == 'both':
            # Command both grippers
            f1 = self._gripper_command('left', position, max_effort)
            f2 = self._gripper_command('right', position, max_effort)
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
        if side not in self._gripper_clients:
            return ActionFuture.failure(f"Gripper {side} not available")

        future = ActionFuture()

        def execute():
            try:
                goal = GripperCommandGoal()
                goal.command.position = position
                goal.command.max_effort = max_effort

                client = self._gripper_clients[side]
                client.send_goal(goal)

                # Wait for result with timeout
                if client.wait_for_result(rospy.Duration(5.0)):
                    result = client.get_result()
                    future._success = True
                    future._result = {
                        'position': result.position,
                        'effort': result.effort,
                        'reached_goal': result.reached_goal
                    }
                else:
                    future._success = False
                    future._error_msg = "Gripper action timed out"
            except Exception as e:
                future._success = False
                future._error_msg = str(e)
            finally:
                future._done.set()

        thread = threading.Thread(target=execute, daemon=True)
        thread.start()

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
        """Get current end effector pose in specified frame."""
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
            # Set the reference frame for pose queries
            mg.set_pose_reference_frame(frame)
            pose_stamped = mg.get_current_pose()
            return pose_stamped.pose
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

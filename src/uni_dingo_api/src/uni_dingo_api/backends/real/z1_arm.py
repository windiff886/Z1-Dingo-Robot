#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Z1ArmBackend - Real Z1 robot arm backend for ArmAPI
"""

import rospy
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from actionlib import SimpleActionClient
from diagnostic_msgs.msg import DiagnosticArray
from typing import List, Optional, Dict, Literal
import threading

from ..api.arm_api import ArmAPI
from ..api.action_future import ActionFuture


class Z1ArmBackend(ArmAPI):
    """
    Real Z1 arm backend.

    Interfaces with the z1_hw_node through ros_control.
    Provides hardware monitoring, safety features, and error handling.
    """

    def __init__(self, side: Literal['left', 'right', 'both'] = 'left',
                 config: dict = None):
        super().__init__(side=side, mode='real', config=config)

        # Hardware state
        self._motor_temperatures = {}  # joint_name: temperature
        self._motor_errors = {}  # joint_name: error_list
        self._arm_errors = {}  # 'left'/'right': error_dict
        self._hardware_ready = {'left': False, 'right': False}
        self._state_lock = threading.Lock()

        # Configuration
        self._config = config or {}

        # Real arm IP addresses (from config or defaults)
        self._arm_ips = {
            'left': config.get('left_arm_ip', '192.168.123.110') if config else '192.168.123.110',
            'right': config.get('right_arm_ip', '192.168.123.111') if config else '192.168.123.111'
        }

        # Real arm UDP ports
        self._arm_ports = {
            'left': {'mcu': 8881, 'sdk': 8871},
            'right': {'mcu': 8882, 'sdk': 8873}
        }

        # Trajectory action clients
        self._traj_clients = {}
        self._init_action_clients()

        # Subscribe to diagnostics
        self._diagnostics_sub = rospy.Subscriber(
            '/diagnostics_agg', DiagnosticArray, self._diagnostics_callback, queue_size=10
        )

        # Gripper position limits
        self._gripper_limits = {
            'open': -1.57,  # rad (full open)
            'close': 0.0    # rad (closed)
        }

        # Joint limits (Z1 specific)
        self._joint_limits = {
            'joint1': (-2.618, 2.618),
            'joint2': (-1.571, 2.094),
            'joint3': (-2.618, 0.0),
            'joint4': (-1.571, 2.618),
            'joint5': (-1.571, 1.571),
            'joint6': (-2.618, 2.618)
        }

        rospy.loginfo(f"Z1ArmBackend initialized for {side} arm(s)")

    def _init_action_clients(self):
        """Initialize trajectory action clients."""
        sides_to_init = ['left', 'right'] if self._side == 'both' else [self._side]

        for side in sides_to_init:
            action_topic = f'/{side}_arm/{side}_z1_joint_traj_controller/follow_joint_trajectory'
            client = SimpleActionClient(action_topic, FollowJointTrajectoryAction)

            # Wait for action server with timeout
            if client.wait_for_server(rospy.Duration(5.0)):
                self._traj_clients[side] = client
                self._hardware_ready[side] = True
                rospy.loginfo(f"Connected to {side} arm trajectory controller")
            else:
                rospy.logwarn(f"Could not connect to {side} arm trajectory controller")
                self._hardware_ready[side] = False

    def _diagnostics_callback(self, msg: DiagnosticArray):
        """Callback for diagnostic updates."""
        with self._state_lock:
            for status in msg.status:
                # Parse Z1 arm diagnostics
                if 'z1' in status.name.lower() or 'arm' in status.name.lower():
                    side = 'left' if 'left' in status.name.lower() else 'right'

                    self._arm_errors[side] = {
                        'level': status.level,  # 0=OK, 1=WARN, 2=ERROR
                        'message': status.message
                    }

                    # Extract motor temperatures from key-value pairs
                    for kv in status.values:
                        if 'temperature' in kv.key.lower():
                            try:
                                self._motor_temperatures[kv.key] = float(kv.value)
                            except ValueError:
                                pass

    # ==================== Hardware Status ====================

    def is_hardware_ready(self, side: str = None) -> bool:
        """Check if arm hardware is ready."""
        side = side or self._side
        if side == 'both':
            return self._hardware_ready['left'] and self._hardware_ready['right']
        return self._hardware_ready.get(side, False)

    def get_motor_temperatures(self, side: str = None) -> Dict[str, float]:
        """
        Get motor temperatures.

        Returns:
            Dict mapping joint names to temperatures (Celsius)
        """
        with self._state_lock:
            if side and side != 'both':
                prefix = f'{side}_'
                return {k: v for k, v in self._motor_temperatures.items()
                        if prefix in k}
            return dict(self._motor_temperatures)

    def has_motor_errors(self, side: str = None) -> bool:
        """Check if any motor has errors."""
        with self._state_lock:
            if side and side != 'both':
                error_info = self._arm_errors.get(side, {})
                return error_info.get('level', 0) >= 2

            for error_info in self._arm_errors.values():
                if error_info.get('level', 0) >= 2:
                    return True
            return False

    def get_arm_errors(self, side: str = None) -> Dict:
        """
        Get arm error information.

        Returns:
            Dict with error details
        """
        with self._state_lock:
            if side and side != 'both':
                return self._arm_errors.get(side, {'level': 0, 'message': 'OK'})
            return dict(self._arm_errors)

    def is_overheating(self, threshold: float = 70.0) -> bool:
        """
        Check if any motor is overheating.

        Args:
            threshold: Temperature threshold in Celsius

        Returns:
            True if any motor exceeds threshold
        """
        temps = self.get_motor_temperatures()
        return any(t > threshold for t in temps.values())

    # ==================== Motion Control ====================

    def move_to_joint_direct(self, positions: List[float], duration: float = 2.0,
                              side: str = None) -> ActionFuture:
        """
        Move to joint positions using trajectory controller directly.

        This bypasses MoveIt for faster execution when collision checking
        is not needed.

        Args:
            positions: List of 6 joint positions (rad)
            duration: Time to reach target (seconds)
            side: Which arm

        Returns:
            ActionFuture
        """
        if not self._enabled:
            return ActionFuture.failure("Arm disabled")

        side = side or self._side
        if side == 'both':
            return ActionFuture.failure("Specify side for direct joint motion")

        # Validate joint positions
        if len(positions) != 6:
            return ActionFuture.failure("Expected 6 joint positions")

        # Check hardware ready
        if not self.is_hardware_ready(side):
            return ActionFuture.failure(f"{side} arm hardware not ready")

        # Check for motor errors
        if self.has_motor_errors(side):
            return ActionFuture.failure(f"{side} arm has motor errors")

        # Validate joint limits
        for i, (pos, (limit_min, limit_max)) in enumerate(
                zip(positions, self._joint_limits.values())):
            if not limit_min <= pos <= limit_max:
                return ActionFuture.failure(
                    f"Joint {i+1} position {pos} exceeds limits [{limit_min}, {limit_max}]")

        client = self._traj_clients.get(side)
        if client is None:
            # Fall back to MoveIt
            return self.move_to_joint(positions, side)

        future = ActionFuture()

        def execute():
            try:
                # Create trajectory goal
                goal = FollowJointTrajectoryGoal()
                goal.trajectory.joint_names = self._joint_names[side]

                point = JointTrajectoryPoint()
                point.positions = positions
                point.velocities = [0.0] * 6
                point.time_from_start = rospy.Duration(duration)
                goal.trajectory.points = [point]

                # Send goal and wait
                client.send_goal(goal)
                result = client.wait_for_result(rospy.Duration(duration + 5.0))

                if result:
                    state = client.get_state()
                    future._success = (state == 3)  # SUCCEEDED
                    future._result = {'state': state}
                    if not future._success:
                        future._error_msg = f"Trajectory execution state: {state}"
                else:
                    future._success = False
                    future._error_msg = "Trajectory execution timeout"

            except Exception as e:
                future._success = False
                future._error_msg = str(e)
                rospy.logerr(f"move_to_joint_direct error: {e}")
            finally:
                future._done.set()

        thread = threading.Thread(target=execute, daemon=True)
        thread.start()

        return future

    def move(self, linear_x: float = 0.0, linear_y: float = 0.0,
             angular_z: float = 0.0) -> None:
        """
        Overridden move to check hardware state first.
        """
        if not self.is_hardware_ready():
            rospy.logwarn("Cannot move - hardware not ready")
            return

        if self.has_motor_errors():
            rospy.logwarn("Cannot move - motor errors detected")
            return

        super().move(linear_x, linear_y, angular_z)

    # ==================== Gripper Control ====================

    def gripper_open(self, side: str = None, position: float = None,
                     speed: float = 0.5) -> ActionFuture:
        """
        Open the gripper.

        Args:
            side: Which arm
            position: Target position (default: full open)
            speed: Opening speed (rad/s)

        Returns:
            ActionFuture
        """
        pos = position if position is not None else self._gripper_limits['open']
        return self._gripper_command_real(side or self._side, pos, speed)

    def gripper_close(self, side: str = None, position: float = None,
                      speed: float = 0.5, max_torque: float = 10.0) -> ActionFuture:
        """
        Close the gripper.

        Args:
            side: Which arm
            position: Target position (default: closed)
            speed: Closing speed (rad/s)
            max_torque: Maximum grasping torque (Nm)

        Returns:
            ActionFuture
        """
        pos = position if position is not None else self._gripper_limits['close']
        return self._gripper_command_real(side or self._side, pos, speed, max_torque)

    def _gripper_command_real(self, side: str, position: float,
                               speed: float = 0.5,
                               max_torque: float = 10.0) -> ActionFuture:
        """Send gripper command for real hardware."""
        if side == 'both':
            f1 = self._gripper_command_real('left', position, speed, max_torque)
            f2 = self._gripper_command_real('right', position, speed, max_torque)

            future = ActionFuture()

            def wait_both():
                f1.wait()
                f2.wait()
                future._success = f1.is_success() and f2.is_success()
                future._done.set()

            thread = threading.Thread(target=wait_both, daemon=True)
            thread.start()
            return future

        future = ActionFuture()

        def execute():
            try:
                # For Z1 gripper, we use trajectory controller
                # The gripper joint is typically included in the arm trajectory controller

                # Create a trajectory with just the gripper
                traj = JointTrajectory()
                traj.header.stamp = rospy.Time.now()
                traj.joint_names = [f'{side}_gripper_joint']

                point = JointTrajectoryPoint()
                point.positions = [position]
                point.velocities = [speed if position < 0 else -speed]

                # Calculate duration based on current position difference
                duration = abs(position - self._gripper_limits['close']) / speed
                point.time_from_start = rospy.Duration(duration)
                traj.points = [point]

                # Publish to gripper controller
                topic = f'/{side}_arm/{side}_gripper_controller/command'
                pub = rospy.Publisher(topic, JointTrajectory, queue_size=1)
                rospy.sleep(0.1)
                pub.publish(traj)

                # Wait for execution
                rospy.sleep(duration + 0.5)

                future._success = True
                future._result = {'position': position}

            except Exception as e:
                future._success = False
                future._error_msg = str(e)
            finally:
                future._done.set()

        thread = threading.Thread(target=execute, daemon=True)
        thread.start()

        return future

    def get_gripper_state(self, side: str = None) -> Optional[Dict]:
        """
        Get gripper state.

        Returns:
            Dict with 'position', 'velocity', 'effort', 'stalled'
        """
        side = side or self._side
        if side == 'both':
            return {
                'left': self.get_gripper_state('left'),
                'right': self.get_gripper_state('right')
            }

        with self._joint_state_lock:
            if self._current_joint_state is None:
                return None

            gripper_name = f'{side}_gripper_joint'
            if gripper_name not in self._current_joint_state.name:
                return None

            idx = self._current_joint_state.name.index(gripper_name)

            result = {
                'position': self._current_joint_state.position[idx],
                'velocity': 0.0,
                'effort': 0.0,
                'stalled': False
            }

            if idx < len(self._current_joint_state.velocity):
                result['velocity'] = self._current_joint_state.velocity[idx]
            if idx < len(self._current_joint_state.effort):
                result['effort'] = self._current_joint_state.effort[idx]
                # Detect stall: high effort but low velocity
                result['stalled'] = (abs(result['effort']) > 5.0 and
                                     abs(result['velocity']) < 0.01)

            return result

    # ==================== Safety ====================

    def emergency_stop(self) -> None:
        """
        Emergency stop - immediately halt all motion.
        """
        rospy.logwarn("Z1 Arm EMERGENCY STOP")

        # Stop MoveIt if active
        self.stop()

        # Cancel any active trajectory goals
        for side, client in self._traj_clients.items():
            try:
                client.cancel_all_goals()
                rospy.loginfo(f"Cancelled {side} arm trajectory goals")
            except Exception as e:
                rospy.logerr(f"Error cancelling {side} arm goals: {e}")

    def clear_errors(self, side: str = None) -> bool:
        """
        Clear arm errors (if possible).

        Note: This requires the z1_controller to support error clearing.

        Returns:
            True if errors were cleared
        """
        # For real hardware, error clearing typically requires
        # communication with the z1_controller
        rospy.loginfo(f"Attempting to clear errors for {side or self._side} arm")

        # The actual implementation would depend on the z1_controller interface
        # For now, we just clear our local error state
        with self._state_lock:
            if side and side != 'both':
                self._arm_errors[side] = {'level': 0, 'message': 'OK'}
            else:
                self._arm_errors = {}

        return True

    # ==================== Configuration ====================

    def get_arm_info(self, side: str = None) -> Dict:
        """
        Get arm hardware information.

        Returns:
            Dict with IP, ports, and status
        """
        side = side or self._side
        if side == 'both':
            return {
                'left': self.get_arm_info('left'),
                'right': self.get_arm_info('right')
            }

        return {
            'ip': self._arm_ips.get(side),
            'ports': self._arm_ports.get(side),
            'hardware_ready': self._hardware_ready.get(side, False),
            'errors': self._arm_errors.get(side, {}),
            'temperatures': self.get_motor_temperatures(side)
        }

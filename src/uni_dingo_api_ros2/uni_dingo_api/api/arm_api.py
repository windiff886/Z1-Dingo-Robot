#!/usr/bin/env python3
"""ArmAPI - Unified interface for robotic arm control (ROS2).

Note: MoveIt2 Python API (moveit_py) is used for motion planning.
The API is initialized lazily when first motion command is issued.
"""

import threading
from typing import Optional, List, Dict, Literal

from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand

from .action_future import ActionFuture


class ArmAPI:
    """Unified API for robotic arm control."""

    def __init__(self, node: Node, side: Literal['left', 'right', 'both'] = 'left',
                 mode: str = 'simulation', config: dict = None):
        self._node = node
        self._side = side
        self._mode = mode
        self._config = config or {}
        self._enabled = True

        self._moveit_initialized = False

        self._planning_groups = {
            'left': 'left_arm_manipulator',
            'right': 'right_arm_manipulator',
        }
        self._ee_links = {
            'left': 'left_link06',
            'right': 'right_link06',
        }
        self._joint_names = {
            'left': [f'left_joint{i}' for i in range(1, 7)],
            'right': [f'right_joint{i}' for i in range(1, 7)],
        }

        self._named_poses = config.get('named_poses', {}) if config else {}

        self._current_joint_state = None
        self._joint_state_lock = threading.Lock()
        self._joint_state_sub = node.create_subscription(
            JointState, '/joint_states', self._joint_state_callback, 10)

        # Gripper action clients (ROS2 action)
        self._gripper_clients = {}
        if side in ['left', 'both']:
            self._gripper_clients['left'] = ActionClient(
                node, GripperCommand, '/left_gripper_controller/gripper_cmd')
        if side in ['right', 'both']:
            self._gripper_clients['right'] = ActionClient(
                node, GripperCommand, '/right_gripper_controller/gripper_cmd')

        self._velocity_scaling = config.get('default_velocity_scaling', 0.3) if config else 0.3
        self._acceleration_scaling = config.get('default_acceleration_scaling', 0.3) if config else 0.3
        self._planning_time = config.get('planning_time', 5.0) if config else 5.0

        node.get_logger().info(f'ArmAPI initialized for {side} arm(s) in {mode} mode')

    def _joint_state_callback(self, msg: JointState):
        with self._joint_state_lock:
            self._current_joint_state = msg

    def _init_moveit(self) -> bool:
        """Initialize MoveIt2 Python interface (lazy)."""
        if self._moveit_initialized:
            return True

        try:
            # moveit_py is the ROS2 Python interface for MoveIt2
            # It requires a running MoveIt2 move_group node
            from moveit.planning import MoveItPy

            self._moveit = MoveItPy(node_name='moveit_py_arm_api')
            self._move_groups = {}

            if self._side in ['left', 'both']:
                self._move_groups['left'] = self._moveit.get_planning_component(
                    self._planning_groups['left'])
            if self._side in ['right', 'both']:
                self._move_groups['right'] = self._moveit.get_planning_component(
                    self._planning_groups['right'])

            self._moveit_initialized = True
            self._node.get_logger().info('MoveIt2 interface initialized')
            return True
        except Exception as e:
            self._node.get_logger().error(f'Failed to initialize MoveIt2: {e}')
            return False

    def move_to_pose(self, pose: Pose, frame: str = 'base_link',
                     side: str = None) -> ActionFuture:
        if not self._enabled:
            return ActionFuture.failure('Arm disabled')
        if not self._init_moveit():
            return ActionFuture.failure('MoveIt2 not available')

        side = side or self._side
        if side == 'both':
            return ActionFuture.failure('Use move_both() for dual-arm motion')

        mg = self._move_groups.get(side)
        if mg is None:
            return ActionFuture.failure(f'No move group for {side}')

        future = ActionFuture()

        def execute():
            try:
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = frame
                pose_stamped.header.stamp = self._node.get_clock().now().to_msg()
                pose_stamped.pose = pose

                mg.set_start_state_to_current_state()
                mg.set_goal_state(pose_stamped_msg=pose_stamped,
                                  pose_link=self._ee_links[side])

                plan_result = mg.plan()
                if plan_result:
                    robot = self._moveit.get_robot_model()
                    success = self._moveit.execute(plan_result.trajectory, controllers=[])
                    future._complete(success)
                else:
                    future._complete(False, error_msg='Motion planning failed')
            except Exception as e:
                future._complete(False, error_msg=str(e))

        threading.Thread(target=execute, daemon=True).start()
        return future

    def move_to_joint(self, positions: List[float], side: str = None) -> ActionFuture:
        if not self._enabled:
            return ActionFuture.failure('Arm disabled')
        if not self._init_moveit():
            return ActionFuture.failure('MoveIt2 not available')

        side = side or self._side
        if side == 'both':
            return ActionFuture.failure('Specify side for joint motion')
        if len(positions) != 6:
            return ActionFuture.failure('Expected 6 joint positions')

        mg = self._move_groups.get(side)
        if mg is None:
            return ActionFuture.failure(f'No move group for {side}')

        future = ActionFuture()

        def execute():
            try:
                from moveit.core.robot_state import RobotState
                robot_model = self._moveit.get_robot_model()
                goal_state = RobotState(robot_model)
                joint_names = self._joint_names[side]
                for name, pos in zip(joint_names, positions):
                    goal_state.joint_positions[name] = pos

                mg.set_start_state_to_current_state()
                mg.set_goal_state(robot_state=goal_state)

                plan_result = mg.plan()
                if plan_result:
                    success = self._moveit.execute(plan_result.trajectory, controllers=[])
                    future._complete(success)
                else:
                    future._complete(False, error_msg='Joint motion planning failed')
            except Exception as e:
                future._complete(False, error_msg=str(e))

        threading.Thread(target=execute, daemon=True).start()
        return future

    def move_to_named(self, name: str, side: str = None) -> ActionFuture:
        if not self._enabled:
            return ActionFuture.failure('Arm disabled')
        if not self._init_moveit():
            return ActionFuture.failure('MoveIt2 not available')

        side = side or self._side
        if side == 'both':
            return self._move_both_to_named(name)

        mg = self._move_groups.get(side)
        if mg is None:
            return ActionFuture.failure(f'No move group for {side}')

        future = ActionFuture()

        def execute():
            try:
                mg.set_start_state_to_current_state()
                mg.set_goal_state(configuration_name=name)
                plan_result = mg.plan()
                if plan_result:
                    success = self._moveit.execute(plan_result.trajectory, controllers=[])
                    future._complete(success)
                else:
                    future._complete(False, error_msg=f'Failed to plan to {name}')
            except Exception as e:
                future._complete(False, error_msg=str(e))

        threading.Thread(target=execute, daemon=True).start()
        return future

    def _move_both_to_named(self, name: str) -> ActionFuture:
        future = ActionFuture()

        def execute():
            results = {}
            threads = []

            def move_single(side_key):
                mg = self._move_groups[side_key]
                mg.set_start_state_to_current_state()
                mg.set_goal_state(configuration_name=name)
                plan_result = mg.plan()
                if plan_result:
                    results[side_key] = self._moveit.execute(
                        plan_result.trajectory, controllers=[])
                else:
                    results[side_key] = False

            for side_key in self._move_groups:
                t = threading.Thread(target=move_single, args=(side_key,))
                threads.append(t)
                t.start()
            for t in threads:
                t.join()

            future._complete(all(results.values()), result=results)

        threading.Thread(target=execute, daemon=True).start()
        return future

    def stop(self, side: str = None) -> None:
        pass  # MoveIt2 stop requires trajectory cancellation

    # Gripper control

    def gripper_open(self, side: str = None, position: float = -1.57) -> ActionFuture:
        return self._gripper_command(side or self._side, position)

    def gripper_close(self, side: str = None, position: float = 0.0) -> ActionFuture:
        return self._gripper_command(side or self._side, position)

    def gripper_set(self, position: float, side: str = None) -> ActionFuture:
        return self._gripper_command(side or self._side, position)

    def _gripper_command(self, side: str, position: float,
                         max_effort: float = 10.0) -> ActionFuture:
        if side == 'both':
            f1 = self._gripper_command('left', position, max_effort)
            f2 = self._gripper_command('right', position, max_effort)
            future = ActionFuture()

            def wait_both():
                f1.wait()
                f2.wait()
                future._complete(f1.is_success() and f2.is_success())

            threading.Thread(target=wait_both, daemon=True).start()
            return future

        if side not in self._gripper_clients:
            return ActionFuture.failure(f'Gripper {side} not available')

        client = self._gripper_clients[side]
        if not client.wait_for_server(timeout_sec=2.0):
            return ActionFuture.failure(f'Gripper {side} action server not available')

        future = ActionFuture()

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = max_effort

        def goal_response_callback(goal_future):
            goal_handle = goal_future.result()
            if not goal_handle.accepted:
                future._complete(False, error_msg='Gripper goal rejected')
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(result_callback)

        def result_callback(result_future):
            result = result_future.result().result
            future._complete(True, result={
                'position': result.position,
                'effort': result.effort,
                'reached_goal': result.reached_goal,
            })

        send_future = client.send_goal_async(goal)
        send_future.add_done_callback(goal_response_callback)
        return future

    # State queries

    def get_joint_states(self, side: str = None) -> Optional[Dict]:
        side = side or self._side
        if side == 'both':
            return {
                'left': self.get_joint_states('left'),
                'right': self.get_joint_states('right'),
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
            return result

    def is_moving(self, side: str = None) -> bool:
        states = self.get_joint_states(side)
        if states is None:
            return False
        if isinstance(states, dict) and 'left' in states:
            return self.is_moving('left') or self.is_moving('right')
        velocities = states.get('velocities', [])
        return any(abs(v) > 0.01 for v in velocities)

    def set_enabled(self, enabled: bool) -> None:
        self._enabled = enabled
        if not enabled:
            self.stop()

    def is_enabled(self) -> bool:
        return self._enabled

    def set_velocity_scaling(self, scaling: float) -> None:
        self._velocity_scaling = max(0.01, min(1.0, scaling))

    def set_acceleration_scaling(self, scaling: float) -> None:
        self._acceleration_scaling = max(0.01, min(1.0, scaling))

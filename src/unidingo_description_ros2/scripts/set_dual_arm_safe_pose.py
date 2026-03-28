#!/usr/bin/env python3
"""在 Gazebo 中为双臂设置安全位姿，并按 ROS1 语义加载控制器。"""

import time

import rclpy
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers, LoadController, SwitchController
from gazebo_msgs.srv import (
    GetModelState,
    GetPhysicsProperties,
    SetModelConfiguration,
    SetPhysicsProperties,
)
from geometry_msgs.msg import Vector3
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from std_srvs.srv import Empty


class SafePoseSetter(Node):
    def __init__(self):
        super().__init__('set_dual_arm_safe_pose')

        double_array_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
        string_array_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)

        self.declare_parameter('model_name', 'uni_dingo_dual_arm')
        self.declare_parameter('timeout_sec', 20.0)
        self.declare_parameter('poll_interval_sec', 0.25)
        self.declare_parameter('set_model', True)
        self.declare_parameter('strict', True)
        self.declare_parameter('retry_interval', 0.5)
        self.declare_parameter('wait_timeout', 10.0)
        self.declare_parameter('model_wait', 10.0)
        self.declare_parameter('controller_wait', 10.0)
        self.declare_parameter('switch_timeout', 2.0)
        self.declare_parameter('unpause', True)
        self.declare_parameter('unpause_after_set_model', False)
        self.declare_parameter('pre_unpause_for_controllers', True)
        self.declare_parameter('pause_on_failure', True)
        self.declare_parameter('disable_gravity_during_controller_start', True)
        self.declare_parameter('restore_gravity_after_safe_pose', True)
        self.declare_parameter(
            'controllers_to_load',
            ['left_arm_controller', 'right_arm_controller'],
            string_array_descriptor,
        )
        self.declare_parameter(
            'start_controllers',
            ['left_arm_controller', 'right_arm_controller'],
            string_array_descriptor,
        )
        self.declare_parameter(
            'left_positions',
            [0.0, 0.0, 0.0, -0.07435, 0.0, 0.0],
            double_array_descriptor,
        )
        self.declare_parameter(
            'right_positions',
            [0.0, 0.0, 0.0, -0.07435, 0.0, 0.0],
            double_array_descriptor,
        )

        self.model_name = self.get_parameter('model_name').value
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.poll_interval_sec = float(self.get_parameter('poll_interval_sec').value)
        self.set_model = bool(self.get_parameter('set_model').value)
        self.strict = bool(self.get_parameter('strict').value)
        self.retry_interval = float(self.get_parameter('retry_interval').value)
        self.wait_timeout = float(self.get_parameter('wait_timeout').value)
        self.model_wait = float(self.get_parameter('model_wait').value)
        self.controller_wait = float(self.get_parameter('controller_wait').value)
        self.switch_timeout = float(self.get_parameter('switch_timeout').value)
        self.unpause = bool(self.get_parameter('unpause').value)
        self.unpause_after_set_model = bool(self.get_parameter('unpause_after_set_model').value)
        self.pre_unpause_for_controllers = bool(
            self.get_parameter('pre_unpause_for_controllers').value
        )
        self.pause_on_failure = bool(self.get_parameter('pause_on_failure').value)
        self.disable_gravity_during_controller_start = bool(
            self.get_parameter('disable_gravity_during_controller_start').value
        )
        self.restore_gravity_after_safe_pose = bool(
            self.get_parameter('restore_gravity_after_safe_pose').value
        )
        self.controllers_to_load = list(self.get_parameter('controllers_to_load').value)
        self.start_controllers = list(self.get_parameter('start_controllers').value)
        self.left_positions = list(self.get_parameter('left_positions').value)
        self.right_positions = list(self.get_parameter('right_positions').value)

        self.get_state_client = self.create_client(GetModelState, '/gazebo/get_model_state')
        self.set_config_client = self.create_client(
            SetModelConfiguration,
            '/gazebo/set_model_configuration',
        )
        self.get_physics_client = self.create_client(
            GetPhysicsProperties,
            '/gazebo/get_physics_properties',
        )
        self.set_physics_client = self.create_client(
            SetPhysicsProperties,
            '/gazebo/set_physics_properties',
        )
        self.unpause_client = self.create_client(Empty, '/gazebo/unpause_physics')
        self.pause_client = self.create_client(Empty, '/gazebo/pause_physics')
        self.list_ctrl_client = self.create_client(
            ListControllers,
            '/controller_manager/list_controllers',
        )
        self.load_ctrl_client = self.create_client(
            LoadController,
            '/controller_manager/load_controller',
        )
        self.switch_ctrl_client = self.create_client(
            SwitchController,
            '/controller_manager/switch_controller',
        )

        self.original_gravity = None
        self.gravity_disabled = False

    def _wait_for_service(self, client, name, timeout_sec) -> bool:
        if client.wait_for_service(timeout_sec=timeout_sec):
            return True
        self.get_logger().error(f'等待 {name} 超时')
        return False

    def _call(self, client, request, timeout_sec=None):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec or self.timeout_sec)
        return future.result()

    def _retry_step(self, step_name, timeout_sec, action) -> bool:
        deadline = time.time() + timeout_sec
        last_detail = ''
        while time.time() < deadline:
            ok, detail = action()
            if ok:
                return True
            last_detail = detail or last_detail
            time.sleep(self.retry_interval)

        if last_detail:
            self.get_logger().error(f'{step_name} 超时: {last_detail}')
        else:
            self.get_logger().error(f'{step_name} 超时')
        return False

    def _handle_failure(self, message) -> int:
        self.get_logger().error(message)
        if self.pause_on_failure:
            self._pause_physics()
        return 1 if self.strict else 0

    def _wait_for_model(self) -> bool:
        def _action():
            request = GetModelState.Request()
            request.model_name = self.model_name
            request.relative_entity_name = 'world'
            response = self._call(self.get_state_client, request, self.poll_interval_sec + 1.0)
            if response and response.success:
                return True, ''
            detail = response.status_message if response else '服务调用失败'
            return False, detail

        return self._retry_step('等待 Gazebo 模型', self.model_wait, _action)

    def _set_safe_pose(self) -> bool:
        request = SetModelConfiguration.Request()
        request.model_name = self.model_name
        request.urdf_param_name = ''
        request.joint_names = [
            'left_joint1',
            'left_joint2',
            'left_joint3',
            'left_joint4',
            'left_joint5',
            'left_joint6',
            'right_joint1',
            'right_joint2',
            'right_joint3',
            'right_joint4',
            'right_joint5',
            'right_joint6',
        ]
        request.joint_positions = [*self.left_positions, *self.right_positions]
        response = self._call(self.set_config_client, request)
        return bool(response and response.success)

    def _get_controller_state(self, controller_name):
        response = self._call(self.list_ctrl_client, ListControllers.Request())
        if response is None:
            return None
        for controller in response.controller:
            if controller.name == controller_name:
                return controller.state
        return None

    def _load_controller(self, controller_name) -> bool:
        request = LoadController.Request()
        request.name = controller_name
        response = self._call(self.load_ctrl_client, request)
        return bool(response and response.ok)

    def _start_controller(self, controller_name) -> bool:
        request = SwitchController.Request()
        request.activate_controllers = [controller_name]
        request.deactivate_controllers = []
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout = Duration(sec=int(self.switch_timeout), nanosec=0)
        response = self._call(self.switch_ctrl_client, request)
        return bool(response and response.ok)

    def _load_controllers(self) -> bool:
        for controller_name in self.controllers_to_load:
            def _action(name=controller_name):
                state = self._get_controller_state(name)
                if state is not None:
                    return True, ''
                if self._load_controller(name):
                    return True, ''
                return False, f'加载控制器 {name} 失败'

            if not self._retry_step(f'加载控制器 {controller_name}', self.controller_wait, _action):
                return False
        return True

    def _start_controllers(self) -> bool:
        for controller_name in self.start_controllers:
            def _action(name=controller_name):
                state = self._get_controller_state(name)
                if state == 'active':
                    return True, ''
                if state is None:
                    return False, f'控制器 {name} 尚未加载'
                if self._start_controller(name):
                    return True, ''
                return False, f'启动控制器 {name} 失败'

            if not self._retry_step(f'启动控制器 {controller_name}', self.controller_wait, _action):
                return False
        return True

    def _get_physics(self):
        return self._call(self.get_physics_client, GetPhysicsProperties.Request())

    def _set_gravity(self, gravity_vector) -> bool:
        physics = self._get_physics()
        if physics is None:
            return False

        request = SetPhysicsProperties.Request()
        request.time_step = physics.time_step
        request.max_update_rate = physics.max_update_rate
        request.gravity = gravity_vector
        request.ode_config = physics.ode_config
        response = self._call(self.set_physics_client, request)
        return bool(response and response.success)

    def _disable_gravity(self) -> bool:
        physics = self._get_physics()
        if physics is None:
            return False
        self.original_gravity = physics.gravity
        if physics.gravity.x == 0.0 and physics.gravity.y == 0.0 and physics.gravity.z == 0.0:
            self.gravity_disabled = True
            return True
        ok = self._set_gravity(Vector3(x=0.0, y=0.0, z=0.0))
        self.gravity_disabled = ok
        return ok

    def _restore_gravity(self) -> bool:
        if not self.gravity_disabled or self.original_gravity is None:
            return True
        return self._set_gravity(self.original_gravity)

    def _unpause_physics(self) -> bool:
        response = self._call(self.unpause_client, Empty.Request())
        return response is not None

    def _pause_physics(self) -> bool:
        response = self._call(self.pause_client, Empty.Request())
        return response is not None

    def run(self) -> int:
        required_services = [
            (self.get_state_client, '/gazebo/get_model_state'),
            (self.set_config_client, '/gazebo/set_model_configuration'),
            (self.list_ctrl_client, '/controller_manager/list_controllers'),
            (self.load_ctrl_client, '/controller_manager/load_controller'),
            (self.switch_ctrl_client, '/controller_manager/switch_controller'),
        ]

        if self.unpause or self.disable_gravity_during_controller_start:
            required_services.extend([
                (self.get_physics_client, '/gazebo/get_physics_properties'),
                (self.set_physics_client, '/gazebo/set_physics_properties'),
                (self.pause_client, '/gazebo/pause_physics'),
                (self.unpause_client, '/gazebo/unpause_physics'),
            ])

        for client, service_name in required_services:
            if not self._wait_for_service(client, service_name, self.wait_timeout):
                return self._handle_failure(f'服务 {service_name} 不可用')

        if not self._wait_for_model():
            return self._handle_failure(f'模型 {self.model_name} 未在超时时间内生成')

        if self.set_model and not self._set_safe_pose():
            return self._handle_failure('设置 Gazebo 安全位姿失败')

        if self.unpause and not self.unpause_after_set_model and self.pre_unpause_for_controllers:
            if self.disable_gravity_during_controller_start and not self._disable_gravity():
                return self._handle_failure('禁用重力失败')
            if not self._unpause_physics():
                return self._handle_failure('Gazebo 预解暂停失败')

        if not self._load_controllers():
            return self._handle_failure('机械臂控制器加载失败')

        if not self._start_controllers():
            return self._handle_failure('机械臂控制器启动失败')

        if self.unpause and self.unpause_after_set_model:
            if not self._unpause_physics():
                return self._handle_failure('Gazebo 解暂停失败')

        if self.restore_gravity_after_safe_pose and not self._restore_gravity():
            return self._handle_failure('恢复重力失败')

        self.get_logger().info('已按 ROS1 语义完成安全位姿与控制器准备流程')
        return 0


def main():
    rclpy.init()
    node = SafePoseSetter()
    try:
        return_code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(return_code)


if __name__ == '__main__':
    main()

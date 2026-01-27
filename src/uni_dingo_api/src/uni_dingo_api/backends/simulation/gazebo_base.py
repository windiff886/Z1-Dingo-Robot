#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GazeboBaseBackend - Gazebo simulation backend for BaseAPI
"""

import rospy
from std_srvs.srv import Empty
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist, Pose2D
import math

from ..api.base_api import BaseAPI
from ..api.action_future import ActionFuture


class GazeboBaseBackend(BaseAPI):
    """
    Gazebo-specific implementation of BaseAPI.

    Adds simulation-specific features like:
    - Reset model pose
    - Pause/unpause simulation
    - Direct model state access
    """

    def __init__(self, config: dict = None):
        super().__init__(mode='simulation', config=config)

        self._model_name = config.get('model_name', 'uni_dingo_dual_arm')

        # Gazebo services (lazy initialization)
        self._pause_srv = None
        self._unpause_srv = None
        self._get_model_state_srv = None
        self._set_model_state_srv = None

        rospy.loginfo("GazeboBaseBackend initialized")

    def _init_gazebo_services(self):
        """Initialize Gazebo service proxies."""
        if self._pause_srv is None:
            try:
                rospy.wait_for_service('/gazebo/pause_physics', timeout=2.0)
                self._pause_srv = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
                self._unpause_srv = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
                self._get_model_state_srv = rospy.ServiceProxy(
                    '/gazebo/get_model_state', GetModelState)
                self._set_model_state_srv = rospy.ServiceProxy(
                    '/gazebo/set_model_state', SetModelState)
            except rospy.ROSException:
                rospy.logwarn("Gazebo services not available")

    # ==================== Simulation Control ====================

    def pause_simulation(self) -> bool:
        """Pause Gazebo physics."""
        self._init_gazebo_services()
        if self._pause_srv:
            try:
                self._pause_srv()
                return True
            except rospy.ServiceException as e:
                rospy.logerr(f"Failed to pause simulation: {e}")
        return False

    def unpause_simulation(self) -> bool:
        """Unpause Gazebo physics."""
        self._init_gazebo_services()
        if self._unpause_srv:
            try:
                self._unpause_srv()
                return True
            except rospy.ServiceException as e:
                rospy.logerr(f"Failed to unpause simulation: {e}")
        return False

    def reset_pose(self, x: float = 0.0, y: float = 0.0, z: float = 0.1,
                   yaw: float = 0.0) -> bool:
        """
        Reset robot pose in simulation.

        Args:
            x, y, z: Position
            yaw: Orientation around Z axis (rad)

        Returns:
            True if successful
        """
        self._init_gazebo_services()
        if self._set_model_state_srv is None:
            return False

        try:
            state = ModelState()
            state.model_name = self._model_name
            state.pose.position.x = x
            state.pose.position.y = y
            state.pose.position.z = z

            # Quaternion from yaw
            state.pose.orientation.x = 0.0
            state.pose.orientation.y = 0.0
            state.pose.orientation.z = math.sin(yaw / 2)
            state.pose.orientation.w = math.cos(yaw / 2)

            state.reference_frame = 'world'

            resp = self._set_model_state_srv(state)
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to reset pose: {e}")
            return False

    def get_gazebo_pose(self) -> Pose:
        """Get model pose directly from Gazebo."""
        self._init_gazebo_services()
        if self._get_model_state_srv is None:
            return None

        try:
            resp = self._get_model_state_srv(self._model_name, 'world')
            if resp.success:
                return resp.pose
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to get model state: {e}")
        return None

    # ==================== Override move_to for simulation ====================

    def teleport_to(self, x: float, y: float, theta: float) -> ActionFuture:
        """
        Instantly teleport to a pose (simulation only).

        Args:
            x, y: Position
            theta: Orientation (rad)

        Returns:
            ActionFuture (immediately complete)
        """
        success = self.reset_pose(x=x, y=y, yaw=theta)
        if success:
            return ActionFuture.success()
        else:
            return ActionFuture.failure("Failed to teleport")

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MoveItArmBackend - MoveIt-based simulation backend for ArmAPI
"""

import rospy
from geometry_msgs.msg import Pose, PoseStamped
from typing import List, Optional

from ...api.arm_api import ArmAPI
from ...api.action_future import ActionFuture


class MoveItArmBackend(ArmAPI):
    """
    MoveIt-specific implementation of ArmAPI.

    This is essentially the same as ArmAPI since ArmAPI already uses MoveIt.
    This class exists to maintain the backend abstraction pattern and can be
    extended with simulation-specific features.
    """

    def __init__(self, side: str = 'left', config: dict = None):
        super().__init__(side=side, mode='simulation', config=config)
        rospy.loginfo(f"MoveItArmBackend initialized for {side}")

    # ==================== Simulation-specific features ====================

    def set_joint_positions_instant(self, positions: List[float],
                                     side: str = None) -> bool:
        """
        Instantly set joint positions (simulation only, no motion planning).

        This can be useful for testing or resetting the arm state.

        Args:
            positions: List of 6 joint positions
            side: Which arm

        Returns:
            True if successful
        """
        # In simulation, we can publish directly to joint trajectory controller
        # For instant positioning, we use a very short duration
        side = side or self._side
        if side == 'both':
            return (self.set_joint_positions_instant(positions, 'left') and
                    self.set_joint_positions_instant(positions, 'right'))

        try:
            from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

            traj = JointTrajectory()
            traj.header.stamp = rospy.Time.now()
            traj.joint_names = self._joint_names[side]

            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start = rospy.Duration(0.1)
            traj.points = [point]

            # Get the appropriate publisher
            topic = f'/{side}_arm/{side}_z1_joint_traj_controller/command'
            pub = rospy.Publisher(topic, JointTrajectory, queue_size=1)
            rospy.sleep(0.1)  # Wait for publisher connection
            pub.publish(traj)

            return True

        except Exception as e:
            rospy.logerr(f"Failed to set instant positions: {e}")
            return False

    def get_planning_scene_objects(self) -> List[str]:
        """Get list of collision objects in planning scene."""
        if self._scene is None:
            self._init_moveit()
        if self._scene:
            return self._scene.get_known_object_names()
        return []

    def add_collision_box(self, name: str, pose: PoseStamped,
                          size: tuple) -> bool:
        """
        Add a collision box to the planning scene.

        Args:
            name: Object name
            pose: Pose of the box center
            size: (x, y, z) dimensions

        Returns:
            True if successful
        """
        if self._scene is None:
            if not self._init_moveit():
                return False

        try:
            self._scene.add_box(name, pose, size)
            return True
        except Exception as e:
            rospy.logerr(f"Failed to add collision box: {e}")
            return False

    def remove_collision_object(self, name: str) -> bool:
        """Remove a collision object from planning scene."""
        if self._scene is None:
            return False

        try:
            self._scene.remove_world_object(name)
            return True
        except Exception as e:
            rospy.logerr(f"Failed to remove object: {e}")
            return False

    def clear_collision_objects(self) -> bool:
        """Remove all collision objects from planning scene."""
        if self._scene is None:
            return False

        try:
            self._scene.remove_world_object()
            return True
        except Exception as e:
            rospy.logerr(f"Failed to clear objects: {e}")
            return False

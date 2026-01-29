#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ActionFuture - Async operation wrapper for ROS actions
"""

import threading
import rospy
import actionlib
from typing import Optional, Callable, Any


class ActionFuture:
    """
    Wraps an asynchronous action, providing a promise-like interface.

    Usage:
        future = arm.move_to_pose(pose)
        future.wait()  # Block until complete

        # Or with callback
        future = arm.move_to_pose(pose)
        future.on_done(lambda result: print("Done!", result))

        # Or check status
        if future.is_done():
            result = future.get_result()
    """

    def __init__(self, action_client: Optional[actionlib.SimpleActionClient] = None,
                 goal: Any = None, timeout: float = 60.0):
        self._client = action_client
        self._goal = goal
        self._timeout = timeout
        self._done = threading.Event()
        self._result = None
        self._success = False
        self._error_msg = ""
        self._callbacks = []
        self._feedback_callbacks = []
        self._cancelled = False

        # For synchronous operations (no action client)
        self._sync_mode = action_client is None

    def _start(self):
        """Start the action (called internally)."""
        if self._sync_mode:
            return

        if self._client and self._goal:
            self._client.send_goal(
                self._goal,
                done_cb=self._on_done,
                feedback_cb=self._on_feedback
            )

    def _on_done(self, state, result):
        """Callback when action completes."""
        self._result = result
        self._success = (state == actionlib.GoalStatus.SUCCEEDED)
        if hasattr(result, 'message'):
            self._error_msg = result.message if not self._success else ""
        self._done.set()

        # Call registered callbacks
        for cb in self._callbacks:
            try:
                cb(self._result, self._success)
            except Exception as e:
                rospy.logwarn(f"ActionFuture callback error: {e}")

    def _on_feedback(self, feedback):
        """Callback for action feedback."""
        for cb in self._feedback_callbacks:
            try:
                cb(feedback)
            except Exception as e:
                rospy.logwarn(f"ActionFuture feedback callback error: {e}")

    def wait(self, timeout: Optional[float] = None) -> 'ActionFuture':
        """
        Block until the action completes.

        Args:
            timeout: Max wait time in seconds (None = use default)

        Returns:
            self for chaining
        """
        wait_time = timeout if timeout is not None else self._timeout

        if self._client:
            # Using action client - wait via client API
            self._client.wait_for_result(rospy.Duration(wait_time))
        else:
            # No action client (manual threading) - wait on event
            self._done.wait(wait_time)

        return self

    def is_done(self) -> bool:
        """Check if the action has completed."""
        return self._done.is_set()

    def is_success(self) -> bool:
        """Check if the action succeeded."""
        return self._success

    def get_result(self) -> Any:
        """Get the action result (blocks if not done)."""
        self.wait()
        return self._result

    def get_error(self) -> str:
        """Get error message if failed."""
        return self._error_msg

    def cancel(self) -> bool:
        """Cancel the action."""
        if self._sync_mode:
            return False
        if self._client and not self._done.is_set():
            self._client.cancel_goal()
            self._cancelled = True
            return True
        return False

    def on_done(self, callback: Callable[[Any, bool], None]) -> 'ActionFuture':
        """
        Register a callback for when action completes.

        Args:
            callback: Function(result, success) to call

        Returns:
            self for chaining
        """
        self._callbacks.append(callback)
        # If already done, call immediately
        if self._done.is_set():
            callback(self._result, self._success)
        return self

    def on_feedback(self, callback: Callable[[Any], None]) -> 'ActionFuture':
        """
        Register a callback for action feedback.

        Args:
            callback: Function(feedback) to call

        Returns:
            self for chaining
        """
        self._feedback_callbacks.append(callback)
        return self

    # Convenience methods for creating completed futures
    @staticmethod
    def success(result: Any = None) -> 'ActionFuture':
        """Create an immediately-successful future."""
        f = ActionFuture()
        f._success = True
        f._result = result
        f._done.set()
        return f

    @staticmethod
    def failure(error_msg: str) -> 'ActionFuture':
        """Create an immediately-failed future."""
        f = ActionFuture()
        f._success = False
        f._error_msg = error_msg
        f._done.set()
        return f

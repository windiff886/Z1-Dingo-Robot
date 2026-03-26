#!/usr/bin/env python3
"""ActionFuture - Async operation wrapper for ROS2 actions."""

import threading
from typing import Optional, Callable, Any

import rclpy
from rclpy.node import Node


class ActionFuture:
    """
    Wraps an asynchronous action, providing a promise-like interface.

    Usage:
        future = arm.move_to_pose(pose)
        future.wait()  # Block until complete

        # Or with callback
        future.on_done(lambda result, success: print("Done!", result))

        # Or check status
        if future.is_done():
            result = future.get_result()
    """

    def __init__(self, timeout: float = 60.0):
        self._timeout = timeout
        self._done = threading.Event()
        self._result = None
        self._success = False
        self._error_msg = ""
        self._callbacks = []
        self._feedback_callbacks = []
        self._cancelled = False
        self._goal_handle = None

    def _set_goal_handle(self, goal_handle):
        """Set the ROS2 action goal handle."""
        self._goal_handle = goal_handle

    def _complete(self, success: bool, result: Any = None, error_msg: str = ""):
        """Mark the future as complete (called internally)."""
        self._result = result
        self._success = success
        self._error_msg = error_msg
        self._done.set()

        for cb in self._callbacks:
            try:
                cb(self._result, self._success)
            except Exception:
                pass

    def _on_feedback(self, feedback):
        """Callback for action feedback."""
        for cb in self._feedback_callbacks:
            try:
                cb(feedback)
            except Exception:
                pass

    def wait(self, timeout: Optional[float] = None) -> 'ActionFuture':
        """Block until the action completes."""
        wait_time = timeout if timeout is not None else self._timeout
        self._done.wait(wait_time)
        return self

    def is_done(self) -> bool:
        return self._done.is_set()

    def is_success(self) -> bool:
        return self._success

    def get_result(self) -> Any:
        self.wait()
        return self._result

    def get_error(self) -> str:
        return self._error_msg

    def cancel(self) -> bool:
        if self._goal_handle and not self._done.is_set():
            self._goal_handle.cancel_goal_async()
            self._cancelled = True
            return True
        return False

    def on_done(self, callback: Callable[[Any, bool], None]) -> 'ActionFuture':
        self._callbacks.append(callback)
        if self._done.is_set():
            callback(self._result, self._success)
        return self

    def on_feedback(self, callback: Callable[[Any], None]) -> 'ActionFuture':
        self._feedback_callbacks.append(callback)
        return self

    @staticmethod
    def success(result: Any = None) -> 'ActionFuture':
        f = ActionFuture()
        f._success = True
        f._result = result
        f._done.set()
        return f

    @staticmethod
    def failure(error_msg: str) -> 'ActionFuture':
        f = ActionFuture()
        f._success = False
        f._error_msg = error_msg
        f._done.set()
        return f

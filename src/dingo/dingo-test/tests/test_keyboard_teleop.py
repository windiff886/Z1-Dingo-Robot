#!/usr/bin/env python3
"""Keyboard command and exit checks with an in-memory ROS publisher."""
import contextlib
import importlib.util
import io
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch


class Twist:
    def __init__(self):
        self.linear = NS(x=0.0, y=0.0, z=0.0)
        self.angular = NS(x=0.0, y=0.0, z=0.0)


ROS = ModuleType('rospy')
ROS.ROSException = RuntimeError
ROS.init_node = Mock()
ROS.get_param = Mock(side_effect=lambda name, default: default)
ROS.logerr = Mock()
MESSAGES = ModuleType('geometry_msgs.msg')
MESSAGES.Twist = Twist
path = Path(__file__).resolve().parents[1] / 'scripts/keyboard_teleop.py'
spec = importlib.util.spec_from_file_location('keyboard_under_test', path)
KEYBOARD = importlib.util.module_from_spec(spec)
with patch.dict(sys.modules, {'rospy': ROS, 'geometry_msgs': ModuleType('geometry_msgs'),
                             'geometry_msgs.msg': MESSAGES}):
    spec.loader.exec_module(KEYBOARD)


class KeyboardTests(unittest.TestCase):
    def setUp(self):
        ROS.on_shutdown = Mock()

    def test_diagonal_speed_is_normalized(self):
        msg = KEYBOARD.make_twist((1, -1, 0), 0.2, 0.5)
        self.assertAlmostEqual((msg.linear.x ** 2 + msg.linear.y ** 2) ** 0.5, 0.2)
        self.assertLess(msg.linear.y, 0)
        self.assertEqual(msg.angular.z, 0)

    def test_parameters_reject_invalid_external_values(self):
        for value in (0, -1, float('nan'), float('inf')):
            with patch.object(ROS, 'get_param', return_value=value):
                with self.assertRaises(ValueError):
                    KEYBOARD.positive_parameter('~linear_speed', 0.2)

    def run_keyboard(self, publisher):
        stdin = Mock()
        stdin.isatty.return_value = True
        stdin.fileno.return_value = 0
        stdin.read.return_value = 'w'
        ROS.Publisher = Mock(return_value=publisher)
        with patch.object(KEYBOARD.sys, 'stdin', stdin), \
                patch.object(KEYBOARD.termios, 'tcgetattr', return_value='saved'), \
                patch.object(KEYBOARD.termios, 'tcsetattr') as restore, \
                patch.object(KEYBOARD.tty, 'setcbreak'), \
                patch.object(KEYBOARD.select, 'select', side_effect=[([stdin], [], []), ([], [], [])]), \
                patch.object(KEYBOARD.time, 'monotonic', return_value=100.0), \
                patch.object(KEYBOARD.time, 'sleep'), \
                contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(KEYBOARD.main(), 0)
        restore.assert_called_once_with(stdin, KEYBOARD.termios.TCSADRAIN, 'saved')

    def test_loop_exit_stops_and_restores_terminal_once(self):
        messages = []
        ROS.is_shutdown = Mock(side_effect=[False, True])
        self.run_keyboard(NS(publish=messages.append))
        self.assertEqual(messages[0].linear.x, 0.2)
        self.assertEqual(len(messages), 4)
        for msg in messages[1:]:
            self.assertEqual((msg.linear.x, msg.linear.y, msg.angular.z), (0, 0, 0))

    def test_ros_shutdown_stops_before_transport_closes(self):
        messages, hooks, closed_attempts = [], [], []
        publisher = NS(closed=False)

        def publish(message):
            if publisher.closed:
                closed_attempts.append(message)
                raise ROS.ROSException('publisher closed')
            messages.append(message)

        def shutdown():
            if not messages:
                return False
            for hook in hooks:
                hook()
            publisher.closed = True
            return True

        publisher.publish = publish
        ROS.on_shutdown.side_effect = hooks.append
        ROS.is_shutdown = shutdown
        self.run_keyboard(publisher)
        self.assertEqual(len(hooks), 1)
        self.assertEqual(messages[0].linear.x, 0.2)
        self.assertEqual(len(messages), 4)
        for msg in messages[1:]:
            self.assertEqual((msg.linear.x, msg.linear.y, msg.angular.z), (0, 0, 0))
        self.assertEqual(len(closed_attempts), 1)


if __name__ == '__main__':
    unittest.main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Keyboard to Joy Node for Dingo Robot

Converts keyboard input to sensor_msgs/Joy messages,
allowing reuse of teleop_twist_joy with keyboard control.

Uses pynput for proper multi-key detection (press & release events).

Key mappings are loaded from YAML config files:
  - keyboard_diff.yaml: for differential drive
  - keyboard_omni.yaml: for omnidirectional drive

Install dependency: pip3 install pynput
"""

import rospy
from sensor_msgs.msg import Joy
import threading

try:
    from pynput import keyboard
except ImportError:
    print("Error: pynput not installed. Run: pip3 install pynput")
    exit(1)


class KeyboardToJoy:
    def __init__(self):
        rospy.init_node('keyboard_to_joy')

        # Load key mappings from params (set by yaml config)
        self.key_bindings = {
            'forward': rospy.get_param('~keys/forward', 'w'),
            'backward': rospy.get_param('~keys/backward', 's'),
            'left': rospy.get_param('~keys/left', 'a'),
            'right': rospy.get_param('~keys/right', 'd'),
            'strafe_left': rospy.get_param('~keys/strafe_left', 'q'),
            'strafe_right': rospy.get_param('~keys/strafe_right', 'e'),
            'enable': rospy.get_param('~keys/enable', ' '),
            'turbo': rospy.get_param('~keys/turbo', 'shift'),
        }

        # Axis mappings (which Joy axis each action maps to)
        self.axis_map = {
            'forward_backward': rospy.get_param('~axis_map/forward_backward', 1),
            'left_right': rospy.get_param('~axis_map/left_right', 0),
            'strafe': rospy.get_param('~axis_map/strafe', -1),  # -1 = disabled
        }

        # Button mappings
        self.button_map = {
            'enable': rospy.get_param('~button_map/enable', 4),
            'turbo': rospy.get_param('~button_map/turbo', 5),
        }

        # Number of axes and buttons in Joy message
        self.num_axes = rospy.get_param('~num_axes', 6)
        self.num_buttons = rospy.get_param('~num_buttons', 12)

        # Publisher
        self.pub = rospy.Publisher('joy', Joy, queue_size=10)
        self.rate = rospy.Rate(rospy.get_param('~rate', 20))

        # Current key states (thread-safe)
        self.pressed_keys = set()
        self.lock = threading.Lock()

        # Keyboard listener
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )

        self.print_help()

    def print_help(self):
        """Print usage instructions"""
        print("""
===========================================
    Dingo Keyboard-to-Joy Controller
===========================================

  Multi-key support enabled (using pynput)

  Key bindings (from config):
    {forward}: Forward
    {backward}: Backward
    {left}: Turn Left
    {right}: Turn Right
    {strafe_left}: Strafe Left (omni only)
    {strafe_right}: Strafe Right (omni only)
    {enable}: Enable (hold)
    {turbo}: Turbo mode (hold with Shift)

  Example: Hold Space + W to move forward
           Hold Space + Shift + W for turbo

  Ctrl+C: Exit
===========================================
""".format(**self.key_bindings))

    def normalize_key(self, key):
        """Convert pynput key to string for comparison"""
        try:
            # Regular character key
            return key.char.lower() if key.char else None
        except AttributeError:
            # Special key (shift, space, etc.)
            if key == keyboard.Key.space:
                return ' '
            elif key == keyboard.Key.shift or key == keyboard.Key.shift_r:
                return 'shift'
            elif key == keyboard.Key.ctrl or key == keyboard.Key.ctrl_r:
                return 'ctrl'
            else:
                return None

    def on_press(self, key):
        """Called when a key is pressed"""
        normalized = self.normalize_key(key)
        if normalized:
            with self.lock:
                self.pressed_keys.add(normalized)

    def on_release(self, key):
        """Called when a key is released"""
        normalized = self.normalize_key(key)
        if normalized:
            with self.lock:
                self.pressed_keys.discard(normalized)

        # Stop on Escape
        if key == keyboard.Key.esc:
            return False

    def key_to_joy(self):
        """Convert current key states to Joy message"""
        joy_msg = Joy()
        joy_msg.header.stamp = rospy.Time.now()
        joy_msg.axes = [0.0] * self.num_axes
        joy_msg.buttons = [0] * self.num_buttons

        with self.lock:
            keys = self.pressed_keys.copy()

        # Forward/Backward axis
        fb_axis = self.axis_map['forward_backward']
        if fb_axis >= 0:
            if self.key_bindings['forward'] in keys:
                joy_msg.axes[fb_axis] = 1.0
            elif self.key_bindings['backward'] in keys:
                joy_msg.axes[fb_axis] = -1.0

        # Left/Right axis (turn or strafe depending on config)
        lr_axis = self.axis_map['left_right']
        if lr_axis >= 0:
            if self.key_bindings['left'] in keys:
                joy_msg.axes[lr_axis] = 1.0
            elif self.key_bindings['right'] in keys:
                joy_msg.axes[lr_axis] = -1.0

        # Strafe axis (omni mode)
        strafe_axis = self.axis_map['strafe']
        if strafe_axis >= 0:
            if self.key_bindings['strafe_left'] in keys:
                joy_msg.axes[strafe_axis] = 1.0
            elif self.key_bindings['strafe_right'] in keys:
                joy_msg.axes[strafe_axis] = -1.0

        # Enable button
        enable_btn = self.button_map['enable']
        if enable_btn >= 0 and self.key_bindings['enable'] in keys:
            joy_msg.buttons[enable_btn] = 1

        # Turbo button
        turbo_btn = self.button_map['turbo']
        if turbo_btn >= 0 and 'shift' in keys:
            joy_msg.buttons[turbo_btn] = 1

        return joy_msg

    def run(self):
        """Main loop"""
        # Start keyboard listener in background thread
        self.listener.start()

        try:
            while not rospy.is_shutdown() and self.listener.running:
                # Publish Joy message
                joy_msg = self.key_to_joy()
                self.pub.publish(joy_msg)
                self.rate.sleep()

        except Exception as e:
            rospy.logerr(f"Error: {e}")
        finally:
            # Stop listener
            self.listener.stop()

            # Publish zero joy to stop robot
            joy_msg = Joy()
            joy_msg.axes = [0.0] * self.num_axes
            joy_msg.buttons = [0] * self.num_buttons
            self.pub.publish(joy_msg)

            print("\nStopped")


if __name__ == '__main__':
    try:
        node = KeyboardToJoy()
        node.run()
    except rospy.ROSInterruptException:
        pass

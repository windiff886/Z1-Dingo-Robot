#!/usr/bin/env python3
"""Publish planar Twist commands from a Linux terminal keyboard."""

import math
import select
import sys
import termios
import time
import tty

import rospy
from geometry_msgs.msg import Twist


# Components are expressed in the base frame: forward, left, counterclockwise.
BINDINGS = {
    "w": (1, 0, 0), "s": (-1, 0, 0),
    "a": (0, 1, 0), "d": (0, -1, 0),
    "q": (0, 0, 1), "e": (0, 0, -1),
    "u": (1, 1, 0), "i": (1, -1, 0),
    "j": (-1, 1, 0), "k": (-1, -1, 0),
}


def positive_parameter(name, default):
    value = float(rospy.get_param(name, default))
    if not math.isfinite(value) or value <= 0:
        raise ValueError("{} must be finite and positive".format(name))
    return value


def make_twist(direction, linear_speed, angular_speed):
    msg = Twist()
    # Normalize diagonal translation to the configured linear speed.
    norm = max(1.0, math.hypot(direction[0], direction[1]))
    msg.linear.x = direction[0] * linear_speed / norm
    msg.linear.y = direction[1] * linear_speed / norm
    msg.angular.z = direction[2] * angular_speed
    return msg


def main():
    rospy.init_node("dingo_keyboard_teleop")
    if not sys.stdin.isatty():
        rospy.logerr("Run this node in an interactive terminal using rosrun.")
        return 1

    linear_speed = positive_parameter("~linear_speed", 0.2)
    angular_speed = positive_parameter("~angular_speed", 0.5)
    publish_rate = positive_parameter("~publish_rate", 20.0)
    key_timeout = positive_parameter("~key_timeout", 0.2)
    topic = rospy.get_param("~cmd_vel_topic", "/dingo/cmd_vel")
    publisher = rospy.Publisher(topic, Twist, queue_size=1)

    def stop():
        for _ in range(3):
            try:
                publisher.publish(Twist())
            except rospy.ROSException:
                break
            time.sleep(0.02)

    rospy.on_shutdown(stop)
    settings = termios.tcgetattr(sys.stdin)
    direction = (0, 0, 0)
    last_key_time = float("-inf")
    period = 1.0 / publish_rate

    print("Dingo keyboard control -> {}".format(topic))
    print("w/s: forward/back | a/d: left/right | q/e: rotate left/right")
    print("u/i: forward diagonals | j/k: backward diagonals")
    print("Space: stop | x or Ctrl-C: stop and exit")
    print("linear={:.2f} m/s, angular={:.2f} rad/s".format(
        linear_speed, angular_speed))
    print("Hold a motion key to repeat; no key input for {:.2f}s stops motion."
          .format(key_timeout))
    print("The OS key-repeat delay may cause a short stop after the first press.")
    sys.stdout.flush()

    try:
        tty.setcbreak(sys.stdin.fileno())
        while not rospy.is_shutdown():
            start = time.monotonic()
            # Drain queued input so the most recent key determines the command.
            while select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1).lower()
                if not key or key in ("x", "\x03"):
                    return 0
                direction = BINDINGS.get(key, (0, 0, 0))
                last_key_time = time.monotonic()
            if time.monotonic() - last_key_time >= key_timeout:
                direction = (0, 0, 0)
            publisher.publish(make_twist(direction, linear_speed, angular_speed))
            # Wall time keeps the keyboard responsive even with paused /clock.
            time.sleep(max(0.0, period - (time.monotonic() - start)))
    finally:
        try:
            stop()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (ValueError, rospy.ROSException) as exc:
        rospy.logerr(str(exc))
        sys.exit(1)

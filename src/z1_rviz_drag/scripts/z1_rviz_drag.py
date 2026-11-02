#!/usr/bin/env python3
"""Drag a TCP marker in RViz and send the pose through the official Z1 SDK.

All kinematics and motion come from the official SDK (Z1Model FK/IK, MoveJ,
MoveL, backToStart, setFsm). This node only converts marker poses and
publishes measured joint states for display.
"""
import fcntl
import math
import os
from pathlib import Path
import sys
import threading

import numpy as np
import rospy
import yaml
from geometry_msgs.msg import Pose
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from sensor_msgs.msg import JointState
from visualization_msgs.msg import (InteractiveMarker, InteractiveMarkerControl,
                                    InteractiveMarkerFeedback, Marker, MarkerArray)

Feedback = InteractiveMarkerFeedback


def pose_to_homo(pose):
    p, o = pose.position, pose.orientation
    q = np.array([o.x, o.y, o.z, o.w], dtype=float)
    n = np.linalg.norm(q)
    x, y, z, w = q / n if n > 1e-9 else (0.0, 0.0, 0.0, 1.0)
    T = np.eye(4)
    T[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def homo_to_pose(T):
    R = np.asarray(T)[:3, :3]
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in np.asarray(T)[:3, 3])
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = x, y, z, w
    return pose


def client_lock(runtime_dir, name):
    # Same lock as z1_official.py client: one SDK owner per arm.
    directory = Path(runtime_dir).expanduser().resolve() / name
    directory.mkdir(parents=True, exist_ok=True)
    lock = (directory / "client.lock").open("a")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock.close()
        raise RuntimeError(f"{name} SDK client is already running (z1_official.py client?)")
    return lock


class Arm:
    def __init__(self, name, config, sdk, runtime_dir):
        self.name, self.prefix = name, name + "_"
        self.lock = client_lock(runtime_dir, name)
        self.has_gripper = bool(config["has_gripper"])
        try:
            self.sdk = sdk.ArmInterface(hasGripper=self.has_gripper,
                                        controllerPort=int(config["controller_port"]),
                                        sdkPort=int(config["sdk_port"]),
                                        load=float(config["load_kg"]))
        except TypeError as error:
            raise RuntimeError("unitree_arm_interface lacks port arguments; build it with "
                               "-DZ1_BUILD_PYTHON=ON (see z1_sdk/README.md)") from error
        self.model = self.sdk._ctrlComp.armModel
        self.invalid = sdk.ArmFSMState.INVALID
        self.mutex = threading.Lock()
        self.target_T = None   # edited target, base frame of this arm
        self.target_q = None   # official IK result for target_T, None if unreachable
        self.version = 0
        self.dragging = self.moved = self.busy = False
        self.message = "waiting for controller"
        self.sdk.loopOn()

    def fsm(self):
        return str(self.sdk.getCurrentState()).split(".")[-1]

    def measured(self):
        if self.sdk.getCurrentState() == self.invalid:
            return None
        return np.array(self.sdk.lowstate.getQ(), dtype=float).reshape(6)

    def fk(self, q):
        return np.array(self.model.forwardKinematics(q, 6), dtype=float)

    def set_target(self, T):
        seed = self.target_q
        if seed is None:
            seed = self.measured()
        q = None
        if seed is not None:
            # Near-seed solution first, then the workspace-checked solver.
            ok, result = self.model.inverseKinematics(T, seed, False)
            if not ok:
                ok, result = self.model.inverseKinematics(T, seed, True)
            q = np.array(result, dtype=float) if ok else None
        with self.mutex:
            self.target_T, self.target_q = T, q
            self.version += 1

    def reset_target(self):
        with self.mutex:
            self.target_T = self.target_q = None
            self.version += 1

    def run(self, label, call, clears_target):
        with self.mutex:
            if self.busy:
                self.message = f"{label} ignored: previous command still running"
                return
            self.busy, version = True, self.version
        self.message = f"{label} running"

        def work():
            ok = False
            try:
                result = call()
                ok = result is None or bool(result)
                self.message = f"{label} " + ("returned" if ok else "rejected by SDK (returned false)")
            except Exception as error:  # report SDK/pybind errors in RViz
                self.message = f"{label} failed: {error}"
            finally:
                with self.mutex:
                    self.busy = False
                    if ok and clears_target and self.version == version:
                        self.target_T = self.target_q = None
            rospy.loginfo("%s: %s", self.name, self.message)

        threading.Thread(target=work, daemon=True).start()

    def close(self):
        self.sdk.loopOff()
        self.lock.close()


def axis_controls():
    controls = []
    for name, (w, x, y, z) in (("x", (1, 1, 0, 0)), ("z", (1, 0, 1, 0)), ("y", (1, 0, 0, 1))):
        n = math.sqrt(w * w + x * x + y * y + z * z)
        for mode, prefix in ((InteractiveMarkerControl.ROTATE_AXIS, "rotate_"),
                             (InteractiveMarkerControl.MOVE_AXIS, "move_")):
            c = InteractiveMarkerControl()
            c.name = prefix + name
            c.orientation.w, c.orientation.x = w / n, x / n
            c.orientation.y, c.orientation.z = y / n, z / n
            c.interaction_mode = mode
            controls.append(c)
    return controls


class DragNode:
    def __init__(self):
        sdk_path = rospy.get_param("~sdk_python_path", "")
        if sdk_path:
            sys.path.insert(0, os.path.expanduser(sdk_path))
        import unitree_arm_interface as sdk
        self.sdk = sdk
        config_path = rospy.get_param("~arms_config")
        with open(os.path.expanduser(config_path), encoding="utf-8") as stream:
            arms_config = yaml.safe_load(stream)["arms"]
        names = rospy.get_param("~arms", ["left", "right"])
        runtime_dir = rospy.get_param("~runtime_dir", os.path.join(
            os.environ.get("XDG_STATE_HOME", os.path.expanduser("~/.local/state")), "z1_sdk"))
        self.send_on_release = rospy.get_param("~send_on_release", True)
        self.movej_speed = float(rospy.get_param("~movej_speed", 0.5))
        self.movel_speed = float(rospy.get_param("~movel_speed", 0.1))
        if not 0 < self.movej_speed <= math.pi or self.movel_speed <= 0:
            raise ValueError("movej_speed must be in (0, pi] rad/s and movel_speed > 0 m/s")

        self.arms = {}
        try:
            for name in names:
                if name not in arms_config:
                    raise ValueError(f"arm '{name}' not in {config_path}")
                self.arms[name] = Arm(name, arms_config[name], sdk, runtime_dir)
        except Exception:
            self.close()
            raise

        self.joint_pub = rospy.Publisher("~arm_joint_states", JointState, queue_size=1)
        self.status_pub = rospy.Publisher("~status", MarkerArray, queue_size=1)
        self.server = InteractiveMarkerServer("z1_rviz_drag")
        self.menu = MenuHandler()
        self.menu.insert("Send MoveJ", callback=lambda fb: self.send(fb, "MoveJ"))
        self.menu.insert("Send MoveL", callback=lambda fb: self.send(fb, "MoveL"))
        self.menu.insert("Reset target to measured", callback=self.on_reset)
        self.menu.insert("backToStart (home)", callback=self.on_home)
        self.menu.insert("PASSIVE", callback=self.on_passive)
        for arm in self.arms.values():
            self.server.insert(self.make_marker(arm), self.on_feedback)
            self.menu.apply(self.server, arm.name)
            self.server.insert(self.make_home_button(arm), self.on_home_button)
        self.server.applyChanges()
        self.shown = {}
        rospy.on_shutdown(self.close)
        rospy.Timer(rospy.Duration(1.0 / float(rospy.get_param("~rate", 20.0))), self.tick)

    def make_marker(self, arm):
        im = InteractiveMarker()
        im.header.frame_id = arm.prefix + "link00"  # SDK arm base frame
        im.name = arm.name
        im.description = f"{arm.name} TCP (right-click for menu)"
        im.scale = 0.15
        im.pose = homo_to_pose(arm.fk(np.zeros(6)))
        ball = Marker(type=Marker.SPHERE)
        ball.scale.x = ball.scale.y = ball.scale.z = 0.035
        ball.color.r, ball.color.g, ball.color.b, ball.color.a = 1.0, 0.6, 0.1, 0.8
        handle = InteractiveMarkerControl()
        handle.name = "menu"
        handle.interaction_mode = InteractiveMarkerControl.MENU
        handle.always_visible = True
        handle.markers.append(ball)
        im.controls.append(handle)
        im.controls.extend(axis_controls())
        return im

    def make_home_button(self, arm):
        # Clickable button behind the arm base, in the arm's link00 frame.
        im = InteractiveMarker()
        im.header.frame_id = arm.prefix + "link00"
        im.name = arm.name + "_home"
        im.scale = 0.1
        im.pose.position.x, im.pose.position.z = -0.15, 0.6
        im.pose.orientation.w = 1.0
        box = Marker(type=Marker.CUBE)
        box.scale.x = box.scale.y = box.scale.z = 0.06
        box.color.r, box.color.g, box.color.b, box.color.a = 0.2, 0.5, 1.0, 0.9
        label = Marker(type=Marker.TEXT_VIEW_FACING)
        label.pose.position.z = 0.07
        label.scale.z = 0.04
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        label.text = f"{arm.name.upper()} HOME"
        button = InteractiveMarkerControl()
        button.name = "home_button"
        button.interaction_mode = InteractiveMarkerControl.BUTTON
        button.always_visible = True
        button.markers.extend([box, label])
        im.controls.append(button)
        return im

    def on_home_button(self, fb):
        if fb.event_type != Feedback.BUTTON_CLICK:
            return
        arm = self.arms.get(fb.marker_name[:-len("_home")])
        if arm is not None:
            arm.reset_target()
            arm.run("backToStart", arm.sdk.backToStart, True)

    def on_feedback(self, fb):
        arm = self.arms.get(fb.marker_name)
        if arm is None:
            return
        if fb.event_type == Feedback.MOUSE_DOWN:
            arm.dragging, arm.moved = True, False
        elif fb.event_type == Feedback.POSE_UPDATE:
            arm.moved = True
            arm.set_target(pose_to_homo(fb.pose))
        elif fb.event_type == Feedback.MOUSE_UP:
            arm.dragging = False
            if arm.moved:
                arm.set_target(pose_to_homo(fb.pose))
                if self.send_on_release:
                    self.send(fb, "MoveJ")

    def send(self, fb, kind):
        arm = self.arms[fb.marker_name]
        with arm.mutex:
            T, q = arm.target_T, arm.target_q
        if T is None:
            arm.message = f"{kind} not sent: no edited target"
            return
        if q is None:
            arm.message = f"{kind} not sent: official IK found no solution"
            return
        if arm.measured() is None:
            arm.message = f"{kind} not sent: no controller feedback"
            return
        posture = np.array(self.sdk.homoToPosture(T), dtype=float)
        if kind == "MoveJ":
            arm.run(kind, lambda: arm.sdk.MoveJ(posture, self.movej_speed), True)
        else:
            arm.run(kind, lambda: arm.sdk.MoveL(posture, self.movel_speed), True)

    def on_reset(self, fb):
        arm = self.arms[fb.marker_name]
        arm.reset_target()
        arm.message = "target reset to measured TCP"

    def on_home(self, fb):
        arm = self.arms[fb.marker_name]
        arm.reset_target()
        arm.run("backToStart", arm.sdk.backToStart, True)

    def on_passive(self, fb):
        # Not serialized with a running Move*: PASSIVE is sent immediately.
        arm = self.arms[fb.marker_name]
        try:
            ok = arm.sdk.setFsm(self.sdk.ArmFSMState.PASSIVE)
            arm.message = "PASSIVE " + ("acknowledged" if ok else "rejected by controller")
        except Exception as error:
            arm.message = f"PASSIVE failed: {error}"

    def tick(self, _event):
        js = JointState()
        js.header.stamp = rospy.Time.now()
        statuses = MarkerArray()
        changed = False
        for i, arm in enumerate(self.arms.values()):
            q = arm.measured()
            if q is not None:
                js.name += [f"{arm.prefix}joint{j}" for j in range(1, 7)]
                js.position += q.tolist()
                if arm.has_gripper:
                    js.name.append(arm.prefix + "jointGripper")
                    js.position.append(float(arm.sdk.lowstate.getGripperQ()))
            with arm.mutex:
                editing = arm.dragging or arm.busy or arm.target_T is not None
                target_ok = arm.target_q is not None
                has_target = arm.target_T is not None
            if q is not None and not editing:
                pose = homo_to_pose(arm.fk(q))
                key = (pose.position.x, pose.position.y, pose.position.z,
                       pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
                last = self.shown.get(arm.name)
                if last is None or max(abs(a - b) for a, b in zip(key, last)) > 1e-4:
                    self.server.setPose(arm.name, pose)
                    self.shown[arm.name] = key
                    changed = True
            elif editing:
                self.shown.pop(arm.name, None)
            statuses.markers.append(self.status_marker(i, arm, has_target, target_ok))
        if changed:
            self.server.applyChanges()
        if js.name:
            self.joint_pub.publish(js)
        self.status_pub.publish(statuses)

    def status_marker(self, i, arm, has_target, target_ok):
        m = Marker(type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
        m.header.frame_id = arm.prefix + "link00"
        m.header.stamp = rospy.Time.now()
        m.ns, m.id = "z1_rviz_drag", i
        m.pose.position.z = 0.75
        m.pose.orientation.w = 1.0
        m.scale.z = 0.035
        target = "no target" if not has_target else ("IK ok" if target_ok else "NO IK")
        busy = " | BUSY" if arm.busy else ""
        m.text = f"{arm.name.upper()} fsm={arm.fsm()} | {target}{busy}\n{arm.message}"
        m.color.a = 1.0
        if has_target and not target_ok:
            m.color.r, m.color.g, m.color.b = 1.0, 0.3, 0.3
        elif has_target:
            m.color.r, m.color.g, m.color.b = 0.4, 1.0, 0.4
        else:
            m.color.r = m.color.g = m.color.b = 1.0
        return m

    def close(self):
        for arm in self.arms.values():
            try:
                arm.close()
            except Exception as error:
                rospy.logwarn("%s: close failed: %s", arm.name, error)
        self.arms = {}


def main():
    rospy.init_node("z1_rviz_drag")
    try:
        DragNode()
    except Exception as error:
        rospy.logfatal("z1_rviz_drag: %s", error)
        sys.exit(1)
    rospy.spin()


if __name__ == "__main__":
    main()

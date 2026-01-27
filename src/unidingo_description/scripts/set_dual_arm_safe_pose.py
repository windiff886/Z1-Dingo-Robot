#!/usr/bin/env python3
import math
import time
import traceback

import rospy
from std_srvs.srv import Empty
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from gazebo_msgs.srv import (
    SetModelConfiguration,
    SpawnModel,
    GetModelState,
    DeleteModel,
    GetPhysicsProperties,
    SetPhysicsProperties,
)
from controller_manager_msgs.srv import ListControllers, SwitchController, LoadController
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Pose, Vector3


def _wait_for_subscribers(pub, timeout_s):
    if timeout_s <= 0.0:
        return True
    end = time.time() + timeout_s
    while not rospy.is_shutdown() and pub.get_num_connections() == 0 and time.time() < end:
        time.sleep(0.1)
    return pub.get_num_connections() > 0


def _build_traj(prefix, positions, time_from_start):
    traj = JointTrajectory()
    traj.joint_names = [
        prefix + "joint1",
        prefix + "joint2",
        prefix + "joint3",
        prefix + "joint4",
        prefix + "joint5",
        prefix + "joint6",
    ]
    point = JointTrajectoryPoint()
    point.positions = positions
    point.time_from_start = rospy.Duration(time_from_start)
    traj.points = [point]
    return traj


def _within_tol(actual, target, tol):
    if len(actual) != len(target):
        return False
    for a, t in zip(actual, target):
        if not math.isfinite(a):
            return False
        if abs(a - t) > tol:
            return False
    return True


def _wait_for_state(topic, timeout_s):
    state = {"msg": None}

    def _cb(msg):
        state["msg"] = msg

    sub = rospy.Subscriber(topic, JointTrajectoryControllerState, _cb, queue_size=1)
    end = time.time() + timeout_s
    while not rospy.is_shutdown() and state["msg"] is None and time.time() < end:
        time.sleep(0.05)
    sub.unregister()
    return state["msg"]


def _service_call(proxy, *args):
    try:
        return proxy(*args), ""
    except (rospy.ServiceException, rospy.ROSException) as exc:
        return None, str(exc)


def _retry_step(step_name, timeout_s, interval_s, action):
    end = time.time() + timeout_s if timeout_s > 0.0 else float("inf")
    attempt = 0
    last_log = 0.0
    last_detail = ""
    while not rospy.is_shutdown() and time.time() < end:
        attempt += 1
        ok, detail = action()
        if ok:
            if attempt > 1:
                rospy.loginfo("%s succeeded after %d attempts.", step_name, attempt)
            return True
        last_detail = detail or last_detail
        now = time.time()
        if now - last_log > 2.0:
            if detail:
                rospy.logwarn("%s retrying: %s", step_name, detail)
            else:
                rospy.logwarn("%s retrying...", step_name)
            last_log = now
        time.sleep(interval_s)
    if last_detail:
        rospy.logerr("%s timed out after %.1fs: %s", step_name, timeout_s, last_detail)
    else:
        rospy.logerr("%s timed out after %.1fs.", step_name, timeout_s)
    return False


def _get_controller_state(list_client, name):
    resp, err = _service_call(list_client)
    if resp is None:
        return None, err
    for controller in resp.controller:
        if controller.name == name:
            return controller.state, ""
    return None, ""


def _wait_for_clock(timeout_s):
    got = {"msg": False}

    def _cb(_msg):
        got["msg"] = True

    sub = rospy.Subscriber("/clock", Clock, _cb, queue_size=1)
    end = time.time() + timeout_s
    while not rospy.is_shutdown() and not got["msg"] and time.time() < end:
        time.sleep(0.05)
    sub.unregister()
    return got["msg"]


def _model_present(get_model_state_client, model_name):
    resp, err = _service_call(get_model_state_client, model_name, "world")
    if resp is None:
        return None, err
    return bool(resp.success), ""


def _get_physics_paused(get_physics_client):
    resp, err = _service_call(get_physics_client)
    if resp is None:
        return None, err
    return bool(resp.pause), ""


def _set_gravity(set_physics_client, physics, gravity):
    resp, err = _service_call(
        set_physics_client,
        physics.time_step,
        physics.max_update_rate,
        gravity,
        physics.ode_config,
    )
    if resp is None:
        return False, err
    return bool(resp.success), resp.status_message


def _set_model_configuration(set_model_client, model_name, joint_names, joint_positions):
    resp, err = _service_call(set_model_client, model_name, "", joint_names, joint_positions)
    if resp is None:
        return False, err
    return bool(resp.success), resp.status_message


def _spawn_model(spawn_client, model_name, robot_description, pose):
    resp, err = _service_call(spawn_client, model_name, robot_description, "", pose, "world")
    if resp is None:
        return False, err
    return bool(resp.success), resp.status_message


def _delete_model(delete_client, model_name):
    resp, err = _service_call(delete_client, model_name)
    if resp is None:
        return False, err
    return bool(resp.success), resp.status_message


def _load_controller(load_client, name):
    resp, err = _service_call(load_client, name)
    if resp is None:
        return False, err
    return bool(resp.ok), ""


def _start_controller(switch_client, name, timeout_s):
    resp, err = _service_call(
        switch_client,
        [name],
        [],
        2,
        True,
        float(timeout_s),
    )
    if resp is None:
        return False, err
    return bool(resp.ok), ""


def _positions_ok(state, target, tol):
    if state is None:
        return False
    positions = state.actual.positions if state.actual.positions else state.desired.positions
    if not positions:
        return False
    return _within_tol(positions, target, tol)


def main():
    rospy.init_node("set_dual_arm_safe_pose", anonymous=True)

    left_positions = rospy.get_param("~left_positions", [0.0, 1.1, -1.1, 1.5707, 0.0, 0.0])
    right_positions = rospy.get_param("~right_positions", [0.0, 1.1, -1.1, 1.5707, 0.0, 0.0])
    time_from_start = float(rospy.get_param("~time_from_start", 1.0))
    wait_timeout = float(rospy.get_param("~wait_timeout", 5.0))
    publish_retries = int(rospy.get_param("~publish_retries", 5))
    state_timeout = float(rospy.get_param("~state_timeout", 1.0))
    max_wait = float(rospy.get_param("~max_wait", 10.0))
    tol = float(rospy.get_param("~tol", 0.02))
    unpause = bool(rospy.get_param("~unpause", False))
    set_model = bool(rospy.get_param("~set_model", True))
    model_name = rospy.get_param("~model_name", "uni_dingo_dual_arm")
    start_controllers = rospy.get_param("~start_controllers", [])
    controller_wait = float(rospy.get_param("~controller_wait", 10.0))
    model_wait = float(rospy.get_param("~model_wait", 10.0))
    unpause_max_wait = float(rospy.get_param("~unpause_max_wait", 60.0))
    unpause_retry_interval = float(rospy.get_param("~unpause_retry_interval", 0.5))
    unpause_after_set_model = bool(rospy.get_param("~unpause_after_set_model", True))
    spawn_model = bool(rospy.get_param("~spawn_model", False))
    spawn_x = float(rospy.get_param("~spawn_x", 0.0))
    spawn_y = float(rospy.get_param("~spawn_y", 0.0))
    spawn_z = float(rospy.get_param("~spawn_z", 0.1))
    spawn_yaw = float(rospy.get_param("~spawn_yaw", 0.0))
    robot_description_param = rospy.get_param("~robot_description_param", "/robot_description")
    controllers_to_load = rospy.get_param("~controllers_to_load", [])
    strict = bool(rospy.get_param("~strict", True))
    retry_interval = float(rospy.get_param("~retry_interval", 0.5))
    delete_existing_model = bool(rospy.get_param("~delete_existing_model", False))
    switch_timeout = float(rospy.get_param("~switch_timeout", 2.0))
    pre_unpause_for_controllers = bool(rospy.get_param("~pre_unpause_for_controllers", True))
    pause_on_failure = bool(rospy.get_param("~pause_on_failure", True))
    disable_gravity_during_controller_start = bool(
        rospy.get_param("~disable_gravity_during_controller_start", True)
    )
    restore_gravity_after_safe_pose = bool(
        rospy.get_param("~restore_gravity_after_safe_pose", True)
    )
    stow_only = bool(rospy.get_param("~stow_only", False))

    left_pub = rospy.Publisher(
        "left_arm/left_z1_joint_traj_controller/command",
        JointTrajectory,
        queue_size=1,
    )
    right_pub = rospy.Publisher(
        "right_arm/right_z1_joint_traj_controller/command",
        JointTrajectory,
        queue_size=1,
    )

    left_traj = _build_traj("left_", left_positions, time_from_start)
    right_traj = _build_traj("right_", right_positions, time_from_start)

    # If sim time is enabled but /clock is not publishing, avoid ROS-time waits.
    use_sim_time = bool(rospy.get_param("/use_sim_time", False))
    clock_active = _wait_for_clock(1.0) if use_sim_time else True
    if use_sim_time and not clock_active:
        rospy.logwarn("use_sim_time is true but /clock is not active; using wall-clock waits.")
    if stow_only and set_model:
        rospy.loginfo("stow_only=true: disabling set_model_configuration step.")
        set_model = False

    rospy.loginfo(
        "Starting safe pose routine (spawn_model=%s, set_model=%s, start_controllers=%s, unpause=%s, strict=%s, stow_only=%s).",
        spawn_model,
        set_model,
        bool(start_controllers),
        unpause,
        strict,
        stow_only,
    )

    get_model_state_srv = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    spawn_model_srv = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
    delete_model_srv = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
    set_model_srv = rospy.ServiceProxy("/gazebo/set_model_configuration", SetModelConfiguration)
    unpause_srv = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
    pause_srv = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
    get_physics_srv = rospy.ServiceProxy("/gazebo/get_physics_properties", GetPhysicsProperties)
    set_physics_srv = rospy.ServiceProxy("/gazebo/set_physics_properties", SetPhysicsProperties)
    list_ctrl_srv = rospy.ServiceProxy("/controller_manager/list_controllers", ListControllers)
    load_ctrl_srv = rospy.ServiceProxy("/controller_manager/load_controller", LoadController)
    switch_ctrl_srv = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)

    if spawn_model:
        rospy.loginfo("Spawning model '%s'...", model_name)
        def _spawn_action():
            present, err = _model_present(get_model_state_srv, model_name)
            if err:
                return False, "get_model_state: %s" % err
            if present:
                if delete_existing_model:
                    ok, detail = _delete_model(delete_model_srv, model_name)
                    if ok:
                        return False, "deleted existing model, retrying spawn"
                    return False, detail or "delete_model failed"
                return True, "model already exists"
            try:
                robot_description = rospy.get_param(robot_description_param)
            except KeyError:
                return False, "robot_description not set"
            if not robot_description:
                return False, "robot_description empty"
            pose = Pose()
            pose.position.x = spawn_x
            pose.position.y = spawn_y
            pose.position.z = spawn_z
            pose.orientation.z = math.sin(spawn_yaw / 2.0)
            pose.orientation.w = math.cos(spawn_yaw / 2.0)
            ok, detail = _spawn_model(spawn_model_srv, model_name, robot_description, pose)
            if ok:
                return True, ""
            return False, detail or "spawn failed"

        if not _retry_step("Spawn model", model_wait, retry_interval, _spawn_action):
            if strict:
                return
            rospy.logwarn("Spawn model failed; continuing because strict=false.")

    # Optionally set joint positions directly in Gazebo before starting controllers
    if set_model:
        rospy.loginfo("Setting model joints to safe pose via Gazebo...")
        joint_names = [
            "left_joint1", "left_joint2", "left_joint3", "left_joint4", "left_joint5", "left_joint6",
            "right_joint1", "right_joint2", "right_joint3", "right_joint4", "right_joint5", "right_joint6",
        ]
        joint_positions = left_positions + right_positions
        def _set_action():
            present, err = _model_present(get_model_state_srv, model_name)
            if err:
                return False, "get_model_state: %s" % err
            if not present:
                return False, "model not present yet"
            ok, detail = _set_model_configuration(set_model_srv, model_name, joint_names, joint_positions)
            if ok:
                return True, ""
            return False, detail or "set_model_configuration failed"

        if not _retry_step("Set model configuration", model_wait, retry_interval, _set_action):
            if strict:
                return
            rospy.logwarn("Set model configuration failed; continuing because strict=false.")

    unpaused = {"value": False}
    original_physics = {"msg": None}

    def _unpause_action():
        paused, err = _get_physics_paused(get_physics_srv)
        if err:
            return False, "get_physics_properties: %s" % err
        if paused is False:
            unpaused["value"] = True
            return True, ""
        resp, err = _service_call(unpause_srv)
        if resp is None:
            return False, "unpause_physics: %s" % err
        paused, err = _get_physics_paused(get_physics_srv)
        if err:
            return False, "get_physics_properties: %s" % err
        if paused is False:
            unpaused["value"] = True
            return True, ""
        return False, "physics still paused"

    def _pause_action():
        paused, err = _get_physics_paused(get_physics_srv)
        if err:
            return False, "get_physics_properties: %s" % err
        if paused is True:
            return True, ""
        resp, err = _service_call(pause_srv)
        if resp is None:
            return False, "pause_physics: %s" % err
        paused, err = _get_physics_paused(get_physics_srv)
        if err:
            return False, "get_physics_properties: %s" % err
        if paused is True:
            return True, ""
        return False, "physics still running"

    def _disable_gravity_action():
        resp, err = _service_call(get_physics_srv)
        if resp is None:
            return False, "get_physics_properties: %s" % err
        if original_physics["msg"] is None:
            original_physics["msg"] = resp
        if resp.gravity.x == 0.0 and resp.gravity.y == 0.0 and resp.gravity.z == 0.0:
            return True, ""
        ok, detail = _set_gravity(set_physics_srv, resp, Vector3(0.0, 0.0, 0.0))
        if ok:
            return True, ""
        return False, detail or "set_physics_properties failed"

    def _restore_gravity_action():
        if original_physics["msg"] is None:
            return True, ""
        resp, err = _service_call(get_physics_srv)
        if resp is None:
            return False, "get_physics_properties: %s" % err
        orig = original_physics["msg"].gravity
        if resp.gravity.x == orig.x and resp.gravity.y == orig.y and resp.gravity.z == orig.z:
            return True, ""
        ok, detail = _set_gravity(set_physics_srv, resp, orig)
        if ok:
            return True, ""
        return False, detail or "restore gravity failed"

    if unpause and unpause_after_set_model:
        if not _retry_step("Unpause Gazebo", unpause_max_wait, unpause_retry_interval, _unpause_action):
            if strict:
                return
            rospy.logwarn("Unpause failed; continuing because strict=false.")

    if unpause and not unpause_after_set_model and pre_unpause_for_controllers:
        if disable_gravity_during_controller_start:
            if not _retry_step("Disable gravity", unpause_max_wait, unpause_retry_interval, _disable_gravity_action):
                if strict:
                    return
                rospy.logwarn("Disable gravity failed; continuing because strict=false.")
        if controllers_to_load or start_controllers:
            if not _retry_step("Pre-unpause for controllers", unpause_max_wait, unpause_retry_interval, _unpause_action):
                if strict:
                    return
                rospy.logwarn("Pre-unpause failed; continuing because strict=false.")

    if controllers_to_load:
        rospy.loginfo("Loading arm controllers (sequential).")
        for name in controllers_to_load:
            def _load_action(ctrl_name=name):
                state, err = _get_controller_state(list_ctrl_srv, ctrl_name)
                if err:
                    return False, "list_controllers: %s" % err
                if state is not None:
                    return True, ""
                ok, detail = _load_controller(load_ctrl_srv, ctrl_name)
                if not ok:
                    return False, detail or "load_controller failed"
                state, err = _get_controller_state(list_ctrl_srv, ctrl_name)
                if err:
                    return False, "list_controllers: %s" % err
                if state is None:
                    return False, "controller not visible yet"
                return True, ""

            if not _retry_step("Load controller %s" % name, controller_wait, retry_interval, _load_action):
                if strict:
                    return
                rospy.logwarn("Load controller %s failed; continuing because strict=false.", name)

    # Optionally start controllers after model has been positioned
    if start_controllers:
        rospy.loginfo("Starting arm controllers (sequential).")
        for name in start_controllers:
            def _start_action(ctrl_name=name):
                state, err = _get_controller_state(list_ctrl_srv, ctrl_name)
                if err:
                    return False, "list_controllers: %s" % err
                if state == "running":
                    return True, ""
                if state is None:
                    return False, "controller not loaded yet"
                ok, detail = _start_controller(switch_ctrl_srv, ctrl_name, switch_timeout)
                if not ok:
                    return False, detail or "switch_controller failed"
                state, err = _get_controller_state(list_ctrl_srv, ctrl_name)
                if err:
                    return False, "list_controllers: %s" % err
                if state == "running":
                    return True, ""
                return False, "controller state is '%s'" % (state or "unknown")

            if not _retry_step("Start controller %s" % name, controller_wait, retry_interval, _start_action):
                if strict:
                    return
                rospy.logwarn("Start controller %s failed; continuing because strict=false.", name)

    if not _wait_for_subscribers(left_pub, wait_timeout):
        if strict:
            rospy.logerr("left arm controller has no subscribers after %.1fs", wait_timeout)
            return
        rospy.logwarn("left arm controller has no subscribers after %.1fs", wait_timeout)
    if not _wait_for_subscribers(right_pub, wait_timeout):
        if strict:
            rospy.logerr("right arm controller has no subscribers after %.1fs", wait_timeout)
            return
        rospy.logwarn("right arm controller has no subscribers after %.1fs", wait_timeout)

    start = time.time()
    sent = 0
    pose_ok = False
    while not rospy.is_shutdown():
        now = time.time()
        if (now - start) > max_wait:
            break

        left_traj.header.stamp = rospy.Time(0)
        right_traj.header.stamp = rospy.Time(0)
        left_pub.publish(left_traj)
        right_pub.publish(right_traj)
        sent += 1

        left_state = _wait_for_state("left_arm/left_z1_joint_traj_controller/state", state_timeout)
        right_state = _wait_for_state("right_arm/right_z1_joint_traj_controller/state", state_timeout)
        if left_state and right_state:
            left_ok = _positions_ok(left_state, left_positions, tol)
            right_ok = _positions_ok(right_state, right_positions, tol)
            if left_ok and right_ok:
                rospy.loginfo("Safe pose accepted by controllers.")
                pose_ok = True
                break

        if sent >= max(publish_retries, 1):
            time.sleep(0.2)
        else:
            time.sleep(0.1)

    if not pose_ok:
        if strict:
            rospy.logerr("Timed out waiting for arms to reach safe pose.")
            if restore_gravity_after_safe_pose:
                _retry_step("Restore gravity", 5.0, 0.5, _restore_gravity_action)
            if unpaused["value"] and pause_on_failure:
                _retry_step("Pause Gazebo", 5.0, 0.5, _pause_action)
            return
        rospy.logwarn("Timed out waiting for arms to reach safe pose.")

    if unpause and not unpause_after_set_model:
        if restore_gravity_after_safe_pose:
            if not _retry_step("Restore gravity", unpause_max_wait, unpause_retry_interval, _restore_gravity_action):
                if strict:
                    return
                rospy.logwarn("Restore gravity failed; continuing because strict=false.")
        if not pose_ok:
            rospy.logwarn("Skip unpause: safe pose not confirmed.")
            return
        if not _retry_step("Unpause Gazebo", unpause_max_wait, unpause_retry_interval, _unpause_action):
            if strict:
                return
            rospy.logwarn("Unpause failed; continuing because strict=false.")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        try:
            rospy.logerr("Unhandled exception in set_dual_arm_safe_pose:\n%s", traceback.format_exc())
        except Exception:
            print(traceback.format_exc())
        raise

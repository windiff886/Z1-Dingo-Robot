#!/usr/bin/env python3
"""Report input freshness separately from the EKF's continuing predictions."""

import math
import threading
import time


class Stream:
    def __init__(self, timeout):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("timeout must be positive and finite")
        self.timeout = timeout
        self.stamp = None
        self.received = None
        self.error = ""

    def update(self, stamp, received, error=""):
        self.stamp = stamp
        self.received = received
        self.error = error

    def inspect(self, ros_now, wall_now, future_tolerance):
        if self.received is None:
            return {"valid": False, "reason": "waiting", "age": None, "arrival_age": None,
                    "stamp": None, "timeout": self.timeout}
        age = ros_now - self.stamp
        arrival_age = wall_now - self.received
        reason = self.error
        if not math.isfinite(self.stamp) or self.stamp <= 0:
            reason = "invalid_timestamp"
        elif age < -future_tolerance:
            reason = "future_timestamp_or_clock_reset"
        elif age > self.timeout or arrival_age > self.timeout:
            reason = "stale"
        return {"valid": not reason, "reason": reason or "fresh",
                "age": age, "arrival_age": arrival_age,
                "stamp": self.stamp, "timeout": self.timeout}


class FeedbackHealth:
    """Pure state logic, also usable in tests without a ROS installation."""

    def __init__(self, wheel_timeout=0.2, imu_timeout=0.2,
                 odom_timeout=0.2, future_tolerance=0.05):
        if not math.isfinite(future_tolerance) or future_tolerance < 0:
            raise ValueError("future_tolerance must be nonnegative and finite")
        self.future_tolerance = future_tolerance
        self.streams = {"wheel": Stream(wheel_timeout), "imu": Stream(imu_timeout),
                        "odom": Stream(odom_timeout)}
        self.lock = threading.Lock()

    def update(self, name, stamp, received, error=""):
        with self.lock:
            self.streams[name].update(stamp, received, error)

    def snapshot(self, ros_now, wall_now):
        with self.lock:
            streams = {name: stream.inspect(ros_now, wall_now, self.future_tolerance)
                       for name, stream in self.streams.items()}
        wheel, imu, odom = (streams[name]["valid"] for name in ("wheel", "imu", "odom"))
        if not odom:
            level, state = 3, "unavailable"
        elif wheel and imu:
            level, state = 0, "ready"
        elif wheel:
            level, state = 1, "wheel_only"
        elif imu:
            level, state = 2, "imu_only"
        else:
            level, state = 3, "prediction_only"
        return level, state, streams


def measurement_error(values, variances):
    if not all(math.isfinite(value) for value in values):
        return "nonfinite_measurement"
    if not all(math.isfinite(value) and value >= 0 for value in variances):
        return "invalid_covariance"
    # Zero means unknown in an IMU message. Require a measured/configured variance
    # rather than treating the EKF's numerical epsilon as an accuracy guarantee.
    if any(value == 0 for value in variances):
        return "unknown_covariance"
    return ""


def main():
    import rospy
    import tf2_ros
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from std_msgs.msg import Header

    rospy.init_node("dingo_feedback_monitor")
    health = FeedbackHealth(
        float(rospy.get_param("~wheel_timeout", 0.2)),
        float(rospy.get_param("~imu_timeout", 0.2)),
        float(rospy.get_param("~odom_timeout", 0.2)),
        float(rospy.get_param("~future_tolerance", 0.05)))
    rate = float(rospy.get_param("~rate", 10.0))
    if not math.isfinite(rate) or rate <= 0:
        raise ValueError("rate must be positive and finite")
    base_frame = rospy.get_param("~base_frame", "base_link")
    odom_frame = rospy.get_param("~odom_frame", "odom")
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    publisher = rospy.Publisher("diagnostics", DiagnosticArray, queue_size=1)

    def transform_error(frame, stamp):
        if not frame:
            return "missing_frame"
        if frame == base_frame:
            return ""
        try:
            if not tf_buffer.can_transform(base_frame, frame, stamp, rospy.Duration(0)):
                return "missing_sensor_transform"
        except (tf2_ros.TransformException, rospy.ROSException):
            return "missing_sensor_transform"
        return ""

    def odom_callback(msg, name):
        twist = msg.twist.twist
        error = measurement_error(
            (twist.linear.x, twist.linear.y, twist.angular.z),
            tuple(msg.twist.covariance[i] for i in (0, 7, 35)))
        error = error or transform_error(msg.child_frame_id, msg.header.stamp)
        if name == "odom" and msg.header.frame_id != odom_frame:
            error = "unexpected_odom_frame"
        health.update(name, msg.header.stamp.to_sec(), time.monotonic(), error)

    def imu_callback(msg):
        error = measurement_error(
            (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z),
            (msg.angular_velocity_covariance[8],))
        if msg.angular_velocity_covariance[0] == -1:
            error = "angular_velocity_unavailable"
        error = error or transform_error(msg.header.frame_id, msg.header.stamp)
        health.update("imu", msg.header.stamp.to_sec(), time.monotonic(), error)

    subscribers = [
        rospy.Subscriber("odom_raw", Odometry, odom_callback, callback_args="wheel", queue_size=10),
        rospy.Subscriber("imu_raw", Imu, imu_callback, queue_size=10),
        rospy.Subscriber("odom", Odometry, odom_callback, callback_args="odom", queue_size=10)]
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        level, state, streams = health.snapshot(now.to_sec(), time.monotonic())
        summary = DiagnosticStatus(level=level, name="dingo/feedback", hardware_id="dingo",
                                   message=state, values=[KeyValue(key="state", value=state)])
        statuses = [summary]
        for name, data in streams.items():
            summary.values.append(KeyValue(key=name + "_valid", value=str(data["valid"]).lower()))
            values = [KeyValue(key=key, value=str(data[key]))
                      for key in ("stamp", "age", "arrival_age", "timeout")]
            statuses.append(DiagnosticStatus(
                level=DiagnosticStatus.OK if data["valid"] else DiagnosticStatus.STALE,
                name="dingo/feedback/" + name, hardware_id="dingo",
                message=data["reason"], values=values))
        publisher.publish(DiagnosticArray(header=Header(stamp=now), status=statuses))
        # Wall time keeps freshness checks running when simulated ROS time pauses.
        time.sleep(1.0 / rate)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import re
import time

import rospy
from std_msgs.msg import Float64


DEFAULT_ARM_TOPICS = [
    "/joint1_controller/command",
    "/joint2_controller/command",
    "/joint3_controller/command",
    "/joint4_controller/command",
    "/joint5_controller/command",
]
DEFAULT_HOME_POSE = [0.0, 0.8, -3.0, -0.5, 0.0]
DEFAULT_GRIPPER_TOPIC = "/r_joint_controller/command"
DEFAULT_GRIPPER_POSITION = -1.20


def parse_float_list(value, expected_len, param_name):
    if isinstance(value, str):
        pieces = [p for p in re.split(r"[\s,]+", value.strip()) if p]
        values = [float(p) for p in pieces]
    else:
        values = [float(v) for v in value]

    if len(values) != expected_len:
        raise ValueError(
            "{} must contain {} values, got {}".format(
                param_name, expected_len, len(values)
            )
        )
    return values


def wait_for_connections(publishers, timeout_s):
    deadline = time.monotonic() + max(timeout_s, 0.0)
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if all(pub.get_num_connections() > 0 for pub in publishers):
            return True
        time.sleep(0.05)
    return all(pub.get_num_connections() > 0 for pub in publishers)


def publish_for_duration(pairs, duration_s, publish_hz):
    sleep_s = 1.0 / max(publish_hz, 1.0)
    deadline = time.monotonic() + max(duration_s, 0.0)
    published = False

    while not rospy.is_shutdown() and (not published or time.monotonic() < deadline):
        for pub, position in pairs:
            pub.publish(Float64(position))
        published = True
        if time.monotonic() < deadline:
            time.sleep(sleep_s)


def main():
    rospy.init_node("arm_home")

    arm_topics = rospy.get_param("~arm_joint_command_topics", DEFAULT_ARM_TOPICS)
    if isinstance(arm_topics, str):
        arm_topics = [p for p in re.split(r"[\s,]+", arm_topics.strip()) if p]
    arm_topics = list(arm_topics)

    home_pose = parse_float_list(
        rospy.get_param("~home_pose", DEFAULT_HOME_POSE),
        len(arm_topics),
        "~home_pose",
    )
    gripper_topic = rospy.get_param("~gripper_command_topic", DEFAULT_GRIPPER_TOPIC)
    gripper_position = float(
        rospy.get_param("~gripper_position", DEFAULT_GRIPPER_POSITION)
    )
    publish_gripper = bool(rospy.get_param("~publish_gripper", True))
    gripper_publish_duration = float(rospy.get_param("~gripper_publish_duration", 0.75))
    arm_publish_duration = float(rospy.get_param("~arm_publish_duration", 1.5))
    publish_hz = float(rospy.get_param("~publish_hz", 10.0))
    connection_timeout = float(rospy.get_param("~connection_timeout", 2.0))

    arm_pubs = [rospy.Publisher(topic, Float64, queue_size=1) for topic in arm_topics]
    gripper_pub = (
        rospy.Publisher(gripper_topic, Float64, queue_size=1)
        if publish_gripper
        else None
    )
    publishers = list(arm_pubs) + ([gripper_pub] if gripper_pub is not None else [])

    if not wait_for_connections(publishers, connection_timeout):
        missing = [
            pub.resolved_name
            for pub in publishers
            if pub.get_num_connections() == 0
        ]
        rospy.logwarn(
            "Timed out waiting for arm controller subscribers; publishing anyway. Missing: %s",
            ", ".join(missing),
        )

    if gripper_pub is not None:
        rospy.loginfo("Opening gripper to %.3f on %s", gripper_position, gripper_topic)
        publish_for_duration(
            [(gripper_pub, gripper_position)], gripper_publish_duration, publish_hz
        )

    rospy.loginfo(
        "Moving arm home on %s: %s",
        ", ".join(arm_topics),
        ["{:.3f}".format(v) for v in home_pose],
    )
    publish_for_duration(list(zip(arm_pubs, home_pose)), arm_publish_duration, publish_hz)
    rospy.loginfo("Arm home command complete.")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        rospy.logerr("arm_home failed: %s", exc)
        raise

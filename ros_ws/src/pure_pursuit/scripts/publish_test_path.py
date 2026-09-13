#!/usr/bin/env python3
"""Publish one latched straight path ahead of the first simulation pose."""
import math
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path


def publish(msg):
    if published[0]:
        return
    q = msg.pose.pose.orientation
    values = (q.x, q.y, q.z, q.w)
    norm = math.hypot(*values)
    if not math.isfinite(norm) or norm < 1e-9:
        return
    x, y, z, w = (v / norm for v in values)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    p = msg.pose.pose.position
    if not msg.header.frame_id or not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
        return
    path = Path()
    path.header = msg.header
    for i in range(21):
        point = PoseStamped()
        point.header = msg.header
        point.pose.position.x = p.x + i * 0.1 * math.cos(yaw)
        point.pose.position.y = p.y + i * 0.1 * math.sin(yaw)
        point.pose.orientation.w = 1.0
        path.poses.append(point)
    pub.publish(path)
    published[0] = True


if __name__ == "__main__":
    rospy.init_node("publish_test_path")
    published = [False]
    pub = rospy.Publisher(rospy.get_param("~path_topic", "/navigation/test_path"),
                          Path, queue_size=1, latch=True)
    sub = rospy.Subscriber(rospy.get_param("~pose_topic", "/base_pose_ground_truth"),
                           Odometry, publish, queue_size=1)
    rospy.spin()

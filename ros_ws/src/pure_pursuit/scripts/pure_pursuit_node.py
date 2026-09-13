#!/usr/bin/env python3
"""ROS Path follower. No TF conversion: path and odometry must share a frame."""
import math
import bisect
import json
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String


class PathProgress:
    """Monotonic arc progress, restricted to a local forward search window."""
    def __init__(self, points):
        self.points = points
        self.arc = [0.0]
        for a, b in zip(points, points[1:]):
            self.arc.append(self.arc[-1] + math.dist(a, b))
        self.progress = 0.0
        self.index = 0
        self.distance = 0.0
        self.previous = None

    def update(self, xy, lookahead):
        if self.previous is not None:
            self.distance += math.dist(xy, self.previous)
        self.previous = xy
        best = (float("inf"), self.progress)
        for i in range(self.index, len(self.points) - 1):
            if self.arc[i] > self.progress + 2 * lookahead:
                break
            a, b = self.points[i], self.points[i + 1]
            dx, dy = b[0] - a[0], b[1] - a[1]
            length2 = dx * dx + dy * dy
            if length2 == 0:
                continue
            t = max(0, min(1, ((xy[0]-a[0])*dx + (xy[1]-a[1])*dy) / length2))
            s = self.arc[i] + t * math.sqrt(length2)
            if s < self.progress:
                continue
            error = math.hypot(xy[0]-a[0]-t*dx, xy[1]-a[1]-t*dy)
            if error < best[0]:
                best = error, s
        self.progress = best[1]
        self.index = min(len(self.points)-2, bisect.bisect_right(self.arc, self.progress)-1)

    def target(self, lookahead):
        s = min(self.arc[-1], self.progress + lookahead)
        i = min(len(self.points)-2, bisect.bisect_right(self.arc, s)-1)
        length = self.arc[i+1] - self.arc[i]
        t = (s-self.arc[i])/length if length else 0
        return tuple(a + t*(b-a) for a, b in zip(self.points[i], self.points[i+1]))

    def complete(self, xy, tolerance):
        return (self.progress >= self.arc[-1] - 2*tolerance
                and self.distance >= 0.9*self.arc[-1]
                and math.dist(xy, self.points[-1]) <= tolerance)


def yaw_from_quaternion(q):
    values = (q.x, q.y, q.z, q.w)
    norm = math.hypot(*values)
    if not all(math.isfinite(v) for v in values) or not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("invalid quaternion")
    x, y, z, w = (v / norm for v in values)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def command(points, pose, lookahead, speed, angular_limit, tolerance):
    """Preserve the reference's waypoint selection and fixed L squared."""
    if not points:
        return 0.0, 0.0
    x, y, yaw = pose
    if math.hypot(points[-1][0] - x, points[-1][1] - y) <= tolerance:
        return 0.0, 0.0
    index = min(range(len(points)),
                key=lambda i: math.hypot(points[i][0] - x, points[i][1] - y))
    while index < len(points) - 1:
        if math.hypot(points[index][0] - x, points[index][1] - y) >= lookahead:
            break
        index += 1
    dx, dy = points[index][0] - x, points[index][1] - y
    local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
    omega = speed * 2 * local_y / (lookahead * lookahead)
    if not math.isfinite(omega):
        return 0.0, 0.0
    return speed, max(-angular_limit, min(angular_limit, omega))


class PurePursuit:
    def __init__(self):
        self.lock = threading.RLock()
        self.pub = rospy.Publisher(rospy.get_param(
            "~cmd_vel_topic", "/navigation/pure_pursuit_cmd_vel"), Twist, queue_size=1)
        self.status_pub = rospy.Publisher("~status", String, queue_size=1)
        self.progress = None
        self.stopped = False
        self.points = []
        self.path_frame = ""
        self.pose = None
        self.pose_frame = ""
        self.pose_stamp = 0.0
        self.received = 0.0
        self.hold = False
        self.reached = False
        rospy.on_shutdown(self.shutdown)
        for name, default in (("look_ahead_distance", 0.7), ("linear_speed", 0.2),
                              ("max_angular_speed", 1.0), ("goal_tolerance", 0.1),
                              ("control_rate", 20.0), ("pose_timeout", 0.5)):
            value = float(rospy.get_param("~" + name, default))
            if not math.isfinite(value) or value <= 0:
                self.pub.publish(Twist())
                raise ValueError(name + " must be finite and positive")
            setattr(self, name, value)
        if not 0 < self.look_ahead_distance * self.look_ahead_distance < float("inf"):
            raise ValueError("look_ahead_distance squared must be finite and positive")
        self.path_sub = rospy.Subscriber(rospy.get_param(
            "~path_topic", "/navigation/test_path"), Path, self.path_callback, queue_size=1)
        self.pose_sub = rospy.Subscriber(rospy.get_param(
            "~pose_topic", "/base_pose_ground_truth"), Odometry,
            self.pose_callback, queue_size=1)

    def path_callback(self, msg):
        with self.lock:
            self.pub.publish(Twist())
            self.points = []
            self.progress = None
            self.hold = True
            self.reached = False
            self.path_frame = msg.header.frame_id
            if self.stopped or not self.path_frame:
                return
            points = []
            for item in msg.poses:
                p = item.pose.position
                if (item.header.frame_id not in ("", self.path_frame)
                        or not all(math.isfinite(v) for v in (p.x, p.y, p.z))):
                    return
                # Orientations are not used for tracking, but reject malformed poses.
                try:
                    yaw_from_quaternion(item.pose.orientation)
                except ValueError:
                    return
                points.append((p.x, p.y))
            self.points = points
            if len(points) >= 4 and math.dist(points[0], points[-1]) < 1e-6:
                self.progress = PathProgress(points)

    def pose_callback(self, msg):
        with self.lock:
            self.pose = None
            p = msg.pose.pose.position
            try:
                yaw = yaw_from_quaternion(msg.pose.pose.orientation)
                if not msg.header.frame_id or not all(
                        math.isfinite(v) for v in (p.x, p.y, p.z)):
                    raise ValueError("invalid position or frame")
            except ValueError:
                self.pub.publish(Twist())
                return
            self.pose = (p.x, p.y, yaw)
            self.pose_frame = msg.header.frame_id
            self.pose_stamp = msg.header.stamp.to_sec()
            self.received = time.monotonic()

    def step(self):
        with self.lock:
            msg = Twist()
            age = rospy.Time.now().to_sec() - self.pose_stamp
            fresh = (0 <= age <= self.pose_timeout
                     and time.monotonic() - self.received <= self.pose_timeout)
            state = "waiting"
            if (not self.stopped and not self.hold and not self.reached
                    and self.points and self.pose is not None and fresh
                    and self.pose_frame == self.path_frame):
                state = "tracking"
                if self.progress is not None:
                    self.progress.update(self.pose[:2], self.look_ahead_distance)
                    self.reached = self.progress.complete(self.pose[:2], self.goal_tolerance)
                    target = self.progress.target(self.look_ahead_distance)
                    # Zero tolerance: completion is gated by full-lap progress above.
                    if not self.reached:
                        msg.linear.x, msg.angular.z = command(
                            [target], self.pose, self.look_ahead_distance,
                            self.linear_speed, self.max_angular_speed, 0.0)
                else:
                    msg.linear.x, msg.angular.z = command(
                        self.points, self.pose, self.look_ahead_distance,
                        self.linear_speed, self.max_angular_speed, self.goal_tolerance)
                    self.reached = math.hypot(self.points[-1][0] - self.pose[0],
                                              self.points[-1][1] - self.pose[1]) <= self.goal_tolerance
            if self.reached:
                state = "reached"
            elif self.pose is None or not fresh:
                state = "invalid_or_stale_pose"
            elif self.hold:
                state = "new_path"
            elif self.pose_frame != self.path_frame:
                state = "frame_mismatch"
            self.hold = False
            self.pub.publish(msg)
            self.status_pub.publish(String(data=json.dumps({
                "state": state,
                "index": self.progress.index if self.progress else 0,
                "progress": self.progress.progress if self.progress else 0,
                "length": self.progress.arc[-1] if self.progress else 0,
                "distance": self.progress.distance if self.progress else 0})))

    def shutdown(self):
        with self.lock:
            self.stopped = True
            self.pub.publish(Twist())

    def run(self):
        # Wall-time watchdog also stops when Gazebo's /clock pauses.
        while not rospy.is_shutdown():
            self.step()
            time.sleep(1.0 / self.control_rate)


if __name__ == "__main__":
    rospy.init_node("pure_pursuit")
    try:
        PurePursuit().run()
    except (ValueError, TypeError) as exc:
        rospy.logfatal(str(exc))
        rospy.signal_shutdown(str(exc))

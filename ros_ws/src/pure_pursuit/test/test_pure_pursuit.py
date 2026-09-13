#!/usr/bin/env python3
import math
import sys
import time
import unittest
from pathlib import Path as FilePath

import rospkg
import rospy
import rostest
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from nav_msgs.msg import Odometry, Path

sys.path.insert(0, str(FilePath(rospkg.RosPack().get_path("pure_pursuit")) / "scripts"))
from pure_pursuit_node import command, yaw_from_quaternion, PathProgress
from publish_interlagos import make_path


class AlgorithmTests(unittest.TestCase):
    def control(self, points, pose=(0, 0, 0), limit=1.0):
        return command(points, pose, 0.7, 0.2, limit, 0.1)

    def test_straight(self):
        self.assertEqual(self.control([(0, 0), (1, 0), (2, 0)]), (0.2, 0.0))

    def test_curve(self):
        self.assertAlmostEqual(self.control([(0, 0), (0.7, 0.3), (1, 1)])[1],
                               0.2 * 2 * 0.3 / 0.7**2)
        self.assertLess(self.control([(0.7, -0.3), (1, -1)])[1], 0)

    def test_global_to_local(self):
        result = self.control([(5, 6), (5, 7)], (5, 5, math.pi / 2))
        self.assertAlmostEqual(result[1], 0, places=12)
        q = Quaternion(0, 0, math.sqrt(0.5), math.sqrt(0.5))
        self.assertAlmostEqual(yaw_from_quaternion(q), math.pi / 2)

    def test_saturation(self):
        self.assertEqual(self.control([(0, 10)], limit=0.3), (0.2, 0.3))
        self.assertEqual(self.control([(0, -10)], limit=0.3), (0.2, -0.3))

    def test_goal(self):
        self.assertEqual(self.control([(0, 0), (0.05, 0)]), (0.0, 0.0))

    def test_empty(self):
        self.assertEqual(self.control([]), (0.0, 0.0))

    def test_invalid_quaternion(self):
        for q in (Quaternion(), Quaternion(0, 0, float("nan"), 1)):
            with self.assertRaises(ValueError):
                yaw_from_quaternion(q)


class RosTests(unittest.TestCase):
    def test_topic_lifecycle(self):
        received = []
        sub = rospy.Subscriber("/pure_pursuit_test/cmd_vel", Twist,
                               lambda m: received.append(m), queue_size=100)
        pose_pub = rospy.Publisher("/pure_pursuit_test/odom", Odometry, queue_size=1)
        path_pub = rospy.Publisher("/pure_pursuit_test/path", Path, queue_size=1)
        deadline = time.monotonic() + 8
        while not (pose_pub.get_num_connections() and path_pub.get_num_connections()):
            self.assertLess(time.monotonic(), deadline, "node did not connect")
            time.sleep(0.02)
        pose = Odometry()
        pose.header.frame_id = "map"
        pose.pose.pose.orientation.w = 1.0
        path = Path()
        path.header.frame_id = "map"
        for x in (0, 1, 2):
            p = PoseStamped()
            p.header.frame_id = "map"
            p.pose.position.x = x
            p.pose.orientation.w = 1
            path.poses.append(p)

        def pump(duration=0.25, stale=False):
            end = time.monotonic() + duration
            while time.monotonic() < end:
                pose.header.stamp = rospy.Time.now() - rospy.Duration(2 if stale else 0)
                pose_pub.publish(pose)
                time.sleep(0.02)

        def is_zero(m):
            return all(v == 0 for v in (m.linear.x, m.linear.y, m.linear.z,
                                        m.angular.x, m.angular.y, m.angular.z))

        def assert_stopped():
            self.assertTrue(received, "no commands received")
            self.assertTrue(all(is_zero(m) for m in received[-3:]))

        pump()
        assert_stopped()  # No path.
        path_pub.publish(path)
        pump()
        self.assertGreater(received[-1].linear.x, 0)
        received.clear()
        path_pub.publish(path)
        pump()
        self.assertTrue(any(is_zero(m) for m in received))  # Replacement stops first.
        self.assertGreater(received[-1].linear.x, 0)
        pump(stale=True)
        assert_stopped()  # Stale header, despite fresh arrivals.
        pump()
        time.sleep(0.7)
        assert_stopped()  # Publisher disappears.
        pose.pose.pose.position.x = 2
        pump()
        assert_stopped()
        pose.pose.pose.position.x = 0
        pump()
        assert_stopped()  # Goal stop stays latched until a new path.
        path_pub.publish(Path())
        pump()
        assert_stopped()
        path_pub.publish(path)
        pump()
        pose.pose.pose.orientation.w = 0
        pump()
        assert_stopped()
        pose.pose.pose.orientation.w = 1
        pose.header.frame_id = "odom"
        pump()
        assert_stopped()
        pose.header.frame_id = "map"
        path.poses[1].pose.position.x = float("nan")
        path_pub.publish(path)
        pump()
        assert_stopped()
        # A closed path starts beside its finish but must execute exactly one lap.
        circuit = make_path()
        closed = Path()
        closed.header.frame_id = "map"
        for x, y in circuit:
            item = PoseStamped()
            item.header.frame_id = "map"
            item.pose.position.x, item.pose.position.y = x, y
            item.pose.orientation.w = 1
            closed.poses.append(item)
        pose.pose.pose.position.x, pose.pose.pose.position.y = circuit[0]
        pump()
        path_pub.publish(closed)
        pump()
        self.assertGreater(received[-1].linear.x, 0, "closed path finished at start")
        for xy in circuit[::3] + [circuit[-1]]:
            pose.pose.pose.position.x, pose.pose.pose.position.y = xy
            pump(0.08)
        pump()
        assert_stopped()
        pose.pose.pose.position.x += 0.5
        pump()
        assert_stopped()  # No second lap, even after pose drifts away.
        sub.unregister()
        pose_pub.unregister()
        path_pub.unregister()


class ClosedPathTests(unittest.TestCase):
    def test_closed_geometry(self):
        points = make_path()
        self.assertEqual(points[0], points[-1])
        self.assertTrue(all(1.499999 <= x <= 8.500001 for p in points for x in p))
        self.assertTrue(all(0.1 <= math.dist(a,b) <= 0.25 for a,b in zip(points,points[1:])))

    def test_no_early_finish(self):
        p = make_path()
        tracker = PathProgress(p)
        tracker.update(p[0], 0.7)
        self.assertFalse(tracker.complete(p[0], 0.1))
        self.assertLess(tracker.progress, 0.1)

    def test_monotonic_index(self):
        p = make_path()
        tracker = PathProgress(p)
        for xy in p[:20]:
            tracker.update(xy, 0.7)
        before = (tracker.index, tracker.progress)
        tracker.update(p[10], 0.7)
        self.assertGreaterEqual(tracker.index, before[0])
        self.assertGreaterEqual(tracker.progress, before[1])

    def test_single_lap_closure(self):
        p = make_path()
        tracker = PathProgress(p)
        previous = 0
        for xy in p[:-1]:
            tracker.update(xy, 0.7)
            self.assertGreaterEqual(tracker.progress, previous)
            self.assertFalse(tracker.complete(xy, 0.1))
            previous = tracker.progress
        tracker.update(p[-1], 0.7)
        self.assertTrue(tracker.complete(p[-1], 0.1))
        self.assertAlmostEqual(tracker.progress, tracker.arc[-1])
        self.assertEqual(tracker.target(0.7), p[-1])


class AllTests(AlgorithmTests, RosTests, ClosedPathTests):
    pass


if __name__ == "__main__":
    rospy.init_node("test_pure_pursuit")
    rostest.rosrun("pure_pursuit", "pure_pursuit", AllTests)

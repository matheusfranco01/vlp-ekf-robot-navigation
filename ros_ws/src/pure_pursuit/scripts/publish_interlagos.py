#!/usr/bin/env python3
"""A drivable Interlagos-inspired loop; normalized layout, no map dependency."""
import bisect
import math

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

# Main straight, tighter first turn, opposite bends, infield, second straight.
CONTROL_POINTS = [(0.15, 0.08), (0.65, 0.08), (0.88, 0.12), (0.93, 0.28),
                  (0.76, 0.36), (0.62, 0.29), (0.51, 0.40), (0.69, 0.55),
                  (0.87, 0.66), (0.82, 0.86), (0.61, 0.92), (0.43, 0.77),
                  (0.49, 0.61), (0.32, 0.53), (0.15, 0.66), (0.08, 0.52),
                  (0.08, 0.28)]


def make_path(spacing=0.15):
    if not 0.1 <= spacing <= 0.25:
        raise ValueError("spacing must be between 0.10 and 0.25 m")
    dense = []
    n = len(CONTROL_POINTS)
    for i in range(n):
        p0, p1, p2, p3 = [CONTROL_POINTS[j % n] for j in (i-1, i, i+1, i+2)]
        for k in range(40):
            t = k / 40
            dense.append(tuple(0.5*((2*b)+(-a+c)*t+(2*a-5*b+4*c-d)*t*t
                                   +(-a+3*b-3*c+d)*t*t*t)
                               for a, b, c, d in zip(p0, p1, p2, p3)))
    # Fit the actual smoothed curve, including spline overshoot, to [1.5, 8.5]^2.
    bounds = [(min(p[k] for p in dense), max(p[k] for p in dense)) for k in (0, 1)]
    dense = [tuple(1.5+7*(p[k]-bounds[k][0])/(bounds[k][1]-bounds[k][0])
                   for k in (0, 1)) for p in dense]
    dense.append(dense[0])
    arc = [0.0]
    for a, b in zip(dense, dense[1:]):
        arc.append(arc[-1]+math.dist(a, b))
    count = math.ceil(arc[-1]/spacing)
    points = []
    for k in range(count):
        s = k*arc[-1]/count
        i = min(len(dense)-2, bisect.bisect_right(arc, s)-1)
        t = (s-arc[i])/(arc[i+1]-arc[i])
        points.append(tuple(a+t*(b-a) for a, b in zip(dense[i], dense[i+1])))
    points.append(points[0])  # Exactly one intentional duplicate marks closure.
    assert all(1.499999 <= v <= 8.500001 for p in points for v in p)
    assert all(math.dist(a, b) > 0.09 for a, b in zip(points, points[1:]))
    return points


def main():
    rospy.init_node("publish_interlagos")
    points = make_path(float(rospy.get_param("~spacing", 0.15)))
    path = Path()
    path.header.frame_id = "map"
    path.header.stamp = rospy.Time.now()
    for i, xy in enumerate(points):
        next_xy = points[(i+1) % (len(points)-1)]
        yaw = math.atan2(next_xy[1]-xy[1], next_xy[0]-xy[0])
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x, pose.pose.position.y = xy
        pose.pose.orientation.z = math.sin(yaw/2)
        pose.pose.orientation.w = math.cos(yaw/2)
        path.poses.append(pose)
    pub = rospy.Publisher(rospy.get_param("~path_topic", "/navigation/test_path"),
                          Path, queue_size=1, latch=True)
    pub.publish(path)
    rospy.loginfo("Closed path: %d points, %.2f m", len(points),
                  sum(math.dist(a, b) for a, b in zip(points, points[1:])))
    rospy.spin()


if __name__ == "__main__":
    main()

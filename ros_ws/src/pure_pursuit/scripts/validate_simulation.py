#!/usr/bin/env python3
"""Bounded Gazebo-only validation; records data and owns its child launches."""
import argparse
import csv
import json
import math
import os
from pathlib import Path as FilePath
import signal
import subprocess
import time

import rosgraph
import rospy
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String


def closest_reference(xy, points):
    best = (float("inf"), 0, points[0])
    for i, (a, b) in enumerate(zip(points, points[1:])):
        dx, dy = b[0]-a[0], b[1]-a[1]
        den = dx*dx+dy*dy
        t = max(0, min(1, ((xy[0]-a[0])*dx+(xy[1]-a[1])*dy)/den)) if den else 0
        ref = (a[0]+t*dx, a[1]+t*dy)
        error = math.dist(xy, ref)
        if error < best[0]:
            best = error, i, ref
    return best


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--timeout", type=float, default=90)
    parser.add_argument("--path-kind", choices=["straight", "interlagos"], default="straight")
    args = parser.parse_args()
    output = FilePath(args.output)
    output.mkdir(parents=True, exist_ok=True)
    rospy.init_node("pure_pursuit_recorder", disable_signals=True)
    master = rosgraph.Master(rospy.get_name())
    rospy.wait_for_service("/controller_manager/list_controllers", timeout=15)
    controllers = rospy.ServiceProxy("/controller_manager/list_controllers", ListControllers)

    def graph(expected):
        pubs, subs, _ = master.getSystemState()
        p = dict(pubs).get("/RosAria/cmd_vel", [])
        s = dict(subs).get("/RosAria/cmd_vel", [])
        assert set(p) == set(expected), ("unexpected publishers", p)
        allowed_subs = {"/gazebo", rospy.get_name()} if expected else {"/gazebo"}
        assert set(s) == allowed_subs, s
        states = {c.name: c.state for c in controllers().controller}
        assert states.get("RosAria") == states.get("p3dx_joint_publisher") == "running", states
        return {"publishers": p, "subscribers": s, "controllers": states}

    before = graph([])
    first = rospy.wait_for_message("/base_pose_ground_truth", Odometry, timeout=10)
    assert first.header.frame_id == "map" and first.child_frame_id == "base_link"
    latest = {"pose": first, "cmd": Twist(), "path": None, "status": {}}
    rospy.Subscriber("/base_pose_ground_truth", Odometry, lambda m: latest.update(pose=m))
    rospy.Subscriber("/RosAria/cmd_vel", Twist, lambda m: latest.update(cmd=m))
    rospy.Subscriber("/navigation/test_path", Path, lambda m: latest.update(path=m))
    rospy.Subscriber("/pure_pursuit/status", String,
                     lambda m: latest.update(status=json.loads(m.data)))
    child = None
    rows = []
    report = {"before": before, "success": False}
    try:
        with (output / "control.log").open("w") as log:
            child = subprocess.Popen(["roslaunch", "pure_pursuit", "simulation_control.launch",
                                      "path_kind:="+args.path_kind],
                                     stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            start = time.monotonic()
            last_xy = None
            distance = 0.0
            stop_start = None
            last_progress = 0
            next_log = start
            while time.monotonic() - start < args.timeout:
                assert child.poll() is None, "control launch exited"
                pose, cmd, path = latest["pose"], latest["cmd"], latest["path"]
                if path is None:
                    time.sleep(0.05)
                    continue
                report["during"] = graph(["/pure_pursuit"])
                p = pose.pose.pose.position
                xy = (p.x, p.y)
                points = [(v.pose.position.x, v.pose.position.y) for v in path.poses]
                error, index, reference = closest_reference(xy, points)
                status = latest["status"]
                progress = status.get("progress", 0)
                if args.path_kind == "interlagos":
                    assert progress >= last_progress, "progress regressed"
                    index = status.get("index", 0)
                last_progress = progress
                if last_xy:
                    distance += math.dist(xy, last_xy)
                last_xy = xy
                speed = math.hypot(pose.twist.twist.linear.x, pose.twist.twist.linear.y)
                assert abs(cmd.linear.x) <= 0.200001 and abs(cmd.angular.z) <= 1.000001
                assert 1.35 < p.x < 8.65 and 1.35 < p.y < 8.65, "left safe region"
                goal_error = math.dist(xy, points[-1])
                zero = cmd.linear.x == 0 and cmd.angular.z == 0
                done = distance > 1.5 and goal_error <= 0.11 and zero and status.get("state") == "reached"
                row = [pose.header.stamp.to_sec(), p.x, p.y, *reference, error,
                       cmd.linear.x, cmd.angular.z, index, distance,
                       status.get("state", "waiting"), speed, pose.twist.twist.angular.z, progress]
                rows.append(row)
                if time.monotonic() >= next_log:
                    print("pose=(%.3f, %.3f) distance=%.2f progress=%.2f error=%.3f state=%s" %
                          (p.x, p.y, distance, progress, error, row[10]), flush=True)
                    next_log = time.monotonic() + 5
                if done and speed < 0.01:
                    if stop_start is None:
                        stop_start = (time.monotonic(), xy)
                    if time.monotonic() - stop_start[0] >= 3:
                        assert math.dist(xy, stop_start[1]) < 0.03 and speed < 0.01
                        report["success"] = True
                        break
                else:
                    stop_start = None
                time.sleep(0.05)
            assert report["success"], "timeout before arrival and stationary hold"
            report.update(initial=rows[0][1:3], final=rows[-1][1:3], distance=distance,
                          mean_error=sum(r[5] for r in rows)/len(rows),
                          rmse=math.sqrt(sum(r[5]**2 for r in rows)/len(rows)),
                          max_error=max(r[5] for r in rows),
                          max_linear_command=max(abs(r[6]) for r in rows),
                          max_angular_command=max(abs(r[7]) for r in rows),
                          max_actual_speed=max(r[11] for r in rows),
                          max_actual_angular=max(abs(r[12]) for r in rows),
                          path_length=sum(math.dist(a,b) for a,b in zip(points,points[1:])),
                          progress=progress, stop_hold_seconds=3, goal_error=goal_error,
                          final_speed=speed, stationary_drift=math.dist(xy, stop_start[1]))
            with (output / "path.csv").open("w") as f:
                writer = csv.writer(f)
                writer.writerow(["x", "y"])
                writer.writerows(points)
    finally:
        if child and child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
        with (output / "trajectory.csv").open("w") as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp", "x", "y", "reference_x", "reference_y", "error",
                             "linear", "angular", "index", "distance", "state",
                             "actual_speed", "actual_angular", "progress"])
            writer.writerows(rows)
        (output / "summary.json").write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2), flush=True)
        rospy.signal_shutdown("validation complete")


if __name__ == "__main__":
    main()

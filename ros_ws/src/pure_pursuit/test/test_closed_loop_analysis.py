#!/usr/bin/env python3
"""Synthetic bags exercise the scientific validity gate, including false completion."""
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, TransformStamped
from nav_msgs.msg import Odometry, Path as RosPath
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

spec=importlib.util.spec_from_file_location('analysis',str(Path(__file__).resolve().parents[1]/'scripts/analyze_closed_loop.py'))
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)


def messages(laps=1):
    rows=[]
    path=RosPath();path.header.frame_id='map'
    for x,y in [(0,0),(1,0),(1,1),(0,1),(0,0)]:
        p=PoseStamped();p.header.frame_id='map';p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.w=1;path.poses.append(p)
    rows.append((a.PATH,path,rospy.Time.from_sec(1)))
    tf=TFMessage()
    for parent,child in [('map','odom'),('odom','base_link_d')]:
        tr=TransformStamped();tr.header.frame_id=parent;tr.child_frame_id=child;tf.transforms.append(tr)
    rows.extend([('/tf',tf,rospy.Time.from_sec(1)),('/tf_static',TFMessage(),rospy.Time.from_sec(1)),('/clock',Clock(),rospy.Time.from_sec(1))])
    for t in np.arange(1,27.01,.02):
        distance=min(max(t-2,0),20)/20*4*laps
        u=distance%4; segment=int(u); f=u-segment
        x,y=[(f,0),(1,f),(1-f,1),(0,1-f)][segment]
        moving=2<=t<22
        for topic in a.FRAMES:
            msg=PoseWithCovarianceStamped() if topic==a.ANN else Odometry()
            msg.header.stamp=rospy.Time.from_sec(float(t));msg.header.frame_id=a.FRAMES[topic][0]
            msg.pose.pose.position.x=x;msg.pose.pose.position.y=y;msg.pose.pose.orientation.w=1
            if topic!=a.ANN:msg.child_frame_id=a.FRAMES[topic][1];msg.twist.twist.linear.x=.2 if moving else 0
            rows.append((topic,msg,msg.header.stamp))
        cmd=Twist();cmd.linear.x=.2 if moving else 0
        rows.append((a.CMD,cmd,rospy.Time.from_sec(float(t))))
        rows.append((a.STATUS,String(data='{"state":"tracking"}' if moving else '{"state":"reached"}'),rospy.Time.from_sec(float(t))))
    return sorted(rows,key=lambda r:r[2].to_sec())


class FakeBag:
    def __init__(self,rows):self.rows=rows
    def __enter__(self):return self
    def __exit__(self,*args):pass
    def read_messages(self):return iter(self.rows)


class AnalysisTests(unittest.TestCase):
    def analyze(self,rows,provenance=True):
        p={'pose_topic':a.EKF,'ekf_inputs':[a.ODOM,a.ANN],'graph_checks':20,'graph_passed':True,'fake_localization_absent':True}
        with patch.object(a.rosbag,'Bag',return_value=FakeBag(rows)):
            return a.analyze('synthetic',p if provenance else None)[0]
    def test_valid_lap(self):self.assertTrue(self.analyze(messages())['completed'])
    def test_reached_after_three_laps_rejected(self):self.assertFalse(self.analyze(messages(3))['completed'])
    def test_missing_ann(self):self.assertFalse(self.analyze([r for r in messages() if r[0]!=a.ANN])['completed'])
    def test_wrong_frame(self):
        rows=messages()
        for t,m,_ in rows:
            if t==a.EKF:m.child_frame_id='base_link'
        self.assertFalse(self.analyze(rows)['completed'])
    def test_duplicate_stamp(self):
        rows=messages();rows.insert(100,rows[100]);self.assertFalse(self.analyze(rows)['completed'])
    def test_missing_provenance(self):self.assertFalse(self.analyze(messages(),False)['completed'])
    def test_gap(self):self.assertFalse(self.analyze([r for r in messages() if r[0]!=a.EKF or not 10<r[2].to_sec()<11])['completed'])
    def test_no_final_hold(self):self.assertFalse(self.analyze([r for r in messages() if r[2].to_sec()<23])['completed'])
    def test_stale_pose(self):
        rows=messages()
        for t,m,s in rows:
            if t==a.STATUS and 10<s.to_sec()<10.1:m.data='{"state":"invalid_or_stale_pose"}'
        self.assertFalse(self.analyze(rows)['completed'])


if __name__=='__main__':unittest.main()

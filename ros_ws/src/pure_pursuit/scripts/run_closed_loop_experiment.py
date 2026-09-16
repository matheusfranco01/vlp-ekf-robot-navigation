#!/usr/bin/env python3
"""One fresh Gazebo-only Experiment C run, bounded and recorded for offline gating.

Ground truth is read only by the Radiance sensor and evaluation/abort observer.
It never supplies EKF/controller pose. Spawn parameters are a fixed prior.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import xmlrpc.client

import rosgraph
import rospy
from controller_manager_msgs.srv import ListControllers
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--output',required=True)
    parser.add_argument('--validation-only',action='store_true')
    args=parser.parse_args(); output=Path(args.output)
    output.mkdir(parents=True,exist_ok=False)
    children=[]; logs=[]; latest={}; diagnostics=[]
    provenance={'graph_passed':False,'graph_checks':0}
    def start(command,name):
        log=(output/(name+'.log')).open('w'); logs.append(log)
        child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        children.append(child); return child
    def stop(child):
        if child.poll() is None:
            os.killpg(child.pid,signal.SIGINT)
            try:child.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGTERM)
                try:child.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
    def receive(msg,topic):latest[topic]=msg
    def xy(msg):return (msg.pose.pose.position.x,msg.pose.pose.position.y)
    def yaw(msg):
        q=msg.pose.pose.orientation
        return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    def master_alive():
        try:return xmlrpc.client.ServerProxy(os.environ.get('ROS_MASTER_URI','http://localhost:11311')).getPid('/experiment_check')[0]==1
        except OSError:return False
    try:
        assert not master_alive(), 'Existing ROS master: refuse to mix experiments'
        base=start(['timeout','--signal=INT','--kill-after=15s','650s','roslaunch','--sigint-timeout=5','--sigterm-timeout=3','pure_pursuit','closed_loop_interlagos_base.launch'],'base')
        deadline=time.monotonic()+40
        while not master_alive():
            assert time.monotonic()<deadline and base.poll() is None,'base startup failed'
            time.sleep(.2)
        rospy.init_node('closed_loop_observer',disable_signals=True)
        master=rosgraph.Master(rospy.get_name())
        for topic in ['/base_pose_ground_truth','/odometry/filtered','/RosAria/odom_']:
            rospy.Subscriber(topic,Odometry,receive,topic)
        rospy.Subscriber('/ann_pose',PoseWithCovarianceStamped,receive,'/ann_pose')
        rospy.Subscriber('/pure_pursuit/status',String,receive,'status')
        rospy.Subscriber('/RosAria/cmd_vel',Twist,receive,'cmd')
        rospy.Subscriber('/diagnostics',DiagnosticArray,lambda m:diagnostics.append(m))
        deadline=time.monotonic()+70
        while not all(t in latest for t in ['/base_pose_ground_truth','/odometry/filtered','/RosAria/odom_','/ann_pose']):
            assert time.monotonic()<deadline,'missing localization input'
            time.sleep(.1)
        initial_time=rospy.Time.now().to_sec(); stationary=[]
        while rospy.Time.now().to_sec()-initial_time<25:
            assert time.monotonic()<deadline,'stationary stabilization timeout'
            stationary.append([rospy.Time.now().to_sec(),*xy(latest['/odometry/filtered']),yaw(latest['/odometry/filtered'])])
            time.sleep(.1)
        g,e=latest['/base_pose_ground_truth'],latest['/odometry/filtered']
        position_error=math.dist(xy(g),xy(e)); yaw_error=abs(math.atan2(math.sin(yaw(g)-yaw(e)),math.cos(yaw(g)-yaw(e))))
        assert position_error<.3 and yaw_error<math.radians(5),(position_error,yaw_error)
        assert diagnostics and all(s.level==0 for s in diagnostics[-1].status), 'EKF diagnostic not OK'
        assert 'Using latest instead' not in (output/'base.log').read_text(), 'TF extrapolation warning'
        settled=[s for s in stationary if s[0]>initial_time+15]
        assert max(math.dist(s[1:3],settled[0][1:3]) for s in settled)<.15,'stationary position drift'
        assert max(abs(s[3]-settled[0][3]) for s in settled)<math.radians(3),'stationary yaw drift'
        for topic,limit in [('/ann_pose',2.),('/RosAria/odom_',.3),('/odometry/filtered',.3)]:
            assert 0<=rospy.Time.now().to_sec()-latest[topic].header.stamp.to_sec()<limit, 'stale stationary '+topic
        provenance['stationary']={'ground_truth':[*xy(g),yaw(g)],'ekf':[*xy(e),yaw(e)],'position_error_m':position_error,'yaw_error_deg':math.degrees(yaw_error)}
        provenance['initial_state']=rospy.get_param('/ekf_se/initial_state')
        provenance['ekf_inputs']=[rospy.get_param('/ekf_se/odom0'),rospy.get_param('/ekf_se/pose0')]
        provenance['ekf_parameters']=rospy.get_param('/ekf_se')
        assert g.header.frame_id==e.header.frame_id=='map' and e.child_frame_id=='base_link_d'
        assert rospy.get_param('/ekf_se/base_link_frame')=='base_link_d'
        states={c.name:c.state for c in rospy.ServiceProxy('/controller_manager/list_controllers',ListControllers)().controller}
        assert states['RosAria']==states['p3dx_joint_publisher']=='running'
        def graph(expected):
            pubs,subs,_=master.getSystemState(); p,s=dict(pubs),dict(subs)
            assert set(p.get('/RosAria/cmd_vel',[]))==set(expected),p.get('/RosAria/cmd_vel')
            assert '/gazebo' in s.get('/RosAria/cmd_vel',[]) and all(n in ['/gazebo',rospy.get_name()] or n.startswith('/record_') for n in s['/RosAria/cmd_vel'])
            assert all(n in ['/estimator_node',rospy.get_name()] or n.startswith('/record_') for n in s.get('/base_pose_ground_truth',[])),s.get('/base_pose_ground_truth')
            assert not any('/fake_localization' in ns for _,ns in pubs+subs)
            if expected:
                assert '/pure_pursuit' in s.get('/odometry/filtered',[])
                assert '/pure_pursuit' not in s.get('/base_pose_ground_truth',[])
                provenance['pose_topic']=rospy.get_param('/pure_pursuit/pose_topic')
            provenance['fake_localization_absent']=True
            provenance['graph_checks']+=1; provenance['last_graph']={'publishers':pubs,'subscribers':subs}
        graph([])
        if args.validation_only:
            def snapshot():
                return {t:[*xy(latest[t]),yaw(latest[t])] for t in ['/base_pose_ground_truth','/odometry/filtered','/RosAria/odom_']}
            motion={'initial':snapshot()}
            pub=rospy.Publisher('/RosAria/cmd_vel',Twist,queue_size=1)
            try:
                time.sleep(.5)
                for phase,v,w,duration in [('forward',.1,0,3),('brake',0,0,2),('rotate',0,.2,2),('stop',0,0,4)]:
                    start_time=rospy.Time.now().to_sec(); wall=time.monotonic()
                    while rospy.Time.now().to_sec()-start_time<duration:
                        assert time.monotonic()-wall<15,'short test clock timeout'
                        pubs,_,_=master.getSystemState()
                        assert dict(pubs).get('/RosAria/cmd_vel')==[rospy.get_name()]
                        command=Twist();command.linear.x=v;command.angular.z=w;pub.publish(command);time.sleep(.04)
                    motion[phase]=snapshot()
            finally:
                for _ in range(10):pub.publish(Twist());time.sleep(.04)
                pub.unregister()
            provenance['short_motion']=motion
            assert motion['stop']['/base_pose_ground_truth'][2]>motion['brake']['/base_pose_ground_truth'][2]+.3
            assert motion['stop']['/odometry/filtered'][2]>motion['brake']['/odometry/filtered'][2]+.3
            assert abs(motion['stop']['/base_pose_ground_truth'][2]-motion['stop']['/odometry/filtered'][2])<math.radians(5)
            provenance['validation_passed']=True
            return
        topics=['/base_pose_ground_truth','/odometry/filtered','/ann_pose','/RosAria/odom_','/RosAria/odom','/navigation/test_path','/RosAria/cmd_vel','/pure_pursuit/status','/tf','/tf_static','/clock','/diagnostics']
        recorder=start(['rosbag','record','-O',str(output/'closed_loop.bag')]+topics,'record')
        time.sleep(2)
        control=start(['roslaunch','--sigint-timeout=5','--sigterm-timeout=3','pure_pursuit','closed_loop_interlagos_controller.launch'],'controller')
        deadline=time.monotonic()+500; reached_at=None; next_graph=time.monotonic()+2; next_print=0
        while time.monotonic()<deadline:
            assert all(p.poll() is None for p in children),'child process exited'
            if time.monotonic()>=next_graph:
                graph(['/pure_pursuit']); next_graph=time.monotonic()+1
            g=latest['/base_pose_ground_truth']; x,y=xy(g)
            assert 1.0<x<9.0 and 1.0<y<9.0,'outside safe physical region'
            status=json.loads(latest['status'].data) if 'status' in latest else {}
            cmd=latest.get('cmd',Twist())
            assert abs(cmd.linear.x)<=.200001 and abs(cmd.angular.z)<=1.000001,'velocity exceeded'
            now=rospy.Time.now().to_sec()
            if status.get('state')=='reached':
                if reached_at is None:reached_at=now
                if now-reached_at>=6:break
            if time.monotonic()>=next_print:
                print(json.dumps({'sim_time':now,'physical_xy':[x,y],'controller':status}),flush=True);next_print=time.monotonic()+10
            time.sleep(.05)
        else:raise RuntimeError('run timeout')
        graph(['/pure_pursuit']); provenance['graph_passed']=True
        stop(control); time.sleep(.3); stop(recorder)
    except Exception as exc:
        provenance['failure']=str(exc)
        raise
    finally:
        for child in reversed(children):stop(child)
        if rospy.core.is_initialized():rospy.signal_shutdown('experiment finished')
        provenance['all_children_stopped']=all(c.poll() is not None for c in children)
        (output/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
        for log in logs:log.close()


if __name__=='__main__':main()

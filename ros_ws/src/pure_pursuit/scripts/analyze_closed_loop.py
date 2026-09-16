#!/usr/bin/env python3
"""Offline Experiment C integrity gate. Invalid attempts never enter aggregation.

Criteria fixed before corrected runs: distance 85--115% of the circuit permits
corner cutting but excludes loops; ordered progress >=95%; final radius 0.5 m
allows localization error, not a second lap. A 3 s hold must have speed <0.01
m/s, angular speed <0.02 rad/s and displacement <0.03 m. Data gap limits are
0.2 s GT/EKF, 0.3 s wheel odometry, 2 s Radiance/ANN, and 0.2 s commands/status.
These are integrity criteria, not accuracy targets. Errors are reported as-is.
A runtime provenance sidecar is mandatory: a bag alone cannot prove inputs.
"""
import argparse
import json
from pathlib import Path

import numpy as np
import rosbag

GT = '/base_pose_ground_truth'
EKF = '/odometry/filtered'
ANN = '/ann_pose'
ODOM = '/RosAria/odom_'
CMD = '/RosAria/cmd_vel'
STATUS = '/pure_pursuit/status'
PATH = '/navigation/test_path'
LIMITS = {GT: (100, .2), EKF: (100, .2), ANN: (20, 2.),
          ODOM: (20, .3), CMD: (100, .2), STATUS: (100, .2)}
FRAMES = {GT: ('map', 'base_link'), EKF: ('map', 'base_link_d'),
          ODOM: ('odom', 'base_link_d'), ANN: ('map', '')}


def stats(values):
    return {'mean_m': float(np.mean(values)),
            'rmse_m': float(np.sqrt(np.mean(values ** 2))),
            'max_m': float(np.max(values))}


def projection(points, path):
    a, ab = path[:-1], np.diff(path, axis=0)
    length = np.linalg.norm(ab, axis=1)
    arc = np.r_[0, np.cumsum(length)]
    errors, progress = [], []
    for p in points:
        u = np.clip(np.sum((p-a)*ab, axis=1)/(length**2), 0, 1)
        distances = np.linalg.norm(a+u[:, None]*ab-p, axis=1)
        i = int(np.argmin(distances))
        errors.append(distances[i])
        progress.append(arc[i]+u[i]*length[i])
    return np.asarray(errors), np.asarray(progress), arc[-1]


def analyze(bag_path, provenance):
    failures, data, status, frames = [], {t: [] for t in LIMITS}, [], {}
    path, topics, tf_edges = None, set(), set()
    def require(condition, message):
        if not condition:
            failures.append(message)
    with rosbag.Bag(str(bag_path)) as bag:
        for topic, msg, stamp in bag.read_messages():
            topics.add(topic)
            if topic in FRAMES:
                frames.setdefault(topic, set()).add((msg.header.frame_id, getattr(msg, 'child_frame_id', '')))
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                norm = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
                require(np.isfinite(norm) and abs(norm-1) < .01, topic+': invalid quaternion')
                yaw = np.arctan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                v = msg.twist.twist if topic != ANN else None
                data[topic].append([msg.header.stamp.to_sec(), p.x, p.y, yaw,
                                    np.hypot(v.linear.x,v.linear.y) if v else 0,
                                    v.angular.z if v else 0])
            elif topic == CMD:
                data[topic].append([stamp.to_sec(), msg.linear.x, msg.angular.z])
                require(all(np.isfinite(x) for x in [msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z]), 'nonfinite command')
                require(msg.linear.y == msg.linear.z == msg.angular.x == msg.angular.y == 0, 'unexpected command axis')
            elif topic == STATUS:
                data[topic].append([stamp.to_sec()])
                status.append(json.loads(msg.data))
            elif topic == PATH:
                require(msg.header.frame_id == 'map' and all(p.header.frame_id in ('','map') for p in msg.poses), 'path frame')
                points = np.asarray([[p.pose.position.x,p.pose.position.y] for p in msg.poses])
                require(path is None or np.array_equal(path, points), 'path replaced during run')
                path = points
            elif topic in ('/tf', '/tf_static'):
                tf_edges.update((t.header.frame_id, t.child_frame_id) for t in msg.transforms)
    require(set(LIMITS) | {PATH,'/tf','/tf_static','/clock'} <= topics, 'missing required topics')
    require({('map','odom'),('odom','base_link_d')} <= tf_edges, 'missing localization TF edges')
    for topic, frame in FRAMES.items():
        require(frames.get(topic) == {frame}, topic+': frame mismatch')
    for topic,(minimum,gap) in LIMITS.items():
        a = np.asarray(data[topic])
        data[topic] = a
        require(len(a) >= minimum, topic+': insufficient samples')
        if len(a):
            require(np.isfinite(a).all() and np.all(a[:,0]>0), topic+': invalid values')
            dt = np.diff(a[:,0])
            # Unstamped command/status messages can share one /clock tick
            # (e.g. the path callback's immediate zero and the timer's zero).
            require(np.all(dt>=0) if topic in (CMD,STATUS) else np.all(dt>0), topic+': nonmonotonic timestamps')
            require(len(dt)>0 and max(dt)<=gap+1e-6, topic+': excessive data gap')
    require(provenance is not None, 'missing runtime provenance')
    if provenance:
        require(provenance.get('pose_topic') == EKF, 'controller pose input not EKF')
        require(provenance.get('ekf_inputs') == [ODOM,ANN], 'unexpected EKF inputs')
        require(provenance.get('graph_checks',0)>0 and provenance.get('graph_passed') is True, 'runtime graph not verified')
        require(provenance.get('fake_localization_absent') is True, 'fake localization not excluded')
    require(path is not None and len(path)>3 and np.isfinite(path).all(), 'invalid path')
    if path is not None and len(path)>3:
        require(np.linalg.norm(path[0]-path[-1])<1e-6 and np.all(np.linalg.norm(np.diff(path,axis=0),axis=1)>0), 'invalid closed circuit')
    result = {'bag': str(bag_path), 'completed': False, 'failures': failures}
    if provenance:
        result['initial_state'] = provenance.get('initial_state')
        result['filter_frequency_hz'] = provenance.get('ekf_parameters', {}).get('frequency')
    # Never interpolate corrupt or incomplete data.
    if failures:
        result['failures'] = sorted(set(failures))
        return result, None
    cmd, gt, ekf = data[CMD], data[GT], data[EKF]
    active = np.any(np.abs(cmd[:,1:])>1e-4,axis=1)
    require(np.any(active), 'no active command interval')
    if not np.any(active):
        return result, None
    t0, last_active = cmd[active][0,0],cmd[active][-1,0]
    reached = [t[0] for t,s in zip(data[STATUS],status) if s.get('state')=='reached' and t[0]>=last_active]
    require(bool(reached), 'controller never reached')
    hold_start = last_active + 1.0  # allow braking, then require a full 3 s hold
    t1 = hold_start + 3.0
    require(last_active-t0>10, 'command interval too short for circuit')
    for topic,a in data.items():
        require(a[0,0]<=t0+.1 and a[-1,0]>=t1, topic+': insufficient interval coverage')
    gt = gt[(gt[:,0]>=t0)&(gt[:,0]<=t1)]
    require(len(gt)>=100 and ekf[0,0]<=t0 and ekf[-1,0]>=t1, 'insufficient synchronized crop')
    if failures:
        return result, None
    tracking, progress, length = projection(gt[:,1:3],path)
    # Unwrap the nearest-segment arc at start/finish; reject teleport-like jumps.
    delta = (np.diff(progress)+length/2)%length-length/2
    physical_progress = float(np.sum(delta))
    travelled = float(np.linalg.norm(np.diff(gt[:,1:3],axis=0),axis=1).sum())
    final_distance = float(np.linalg.norm(gt[-1,1:3]-path[-1]))
    hold = gt[gt[:,0]>=hold_start]
    require(.85*length<=travelled<=1.15*length, 'physical distance is not approximately one lap')
    require(physical_progress>=.95*length and physical_progress<=1.05*length, 'physical ordered progress is not one lap')
    require(np.max(np.abs(delta))<=.2, 'physical progress discontinuity')
    require(final_distance<=.5, 'physical finish too far away')
    require(len(hold)>=100 and hold[-1,0]-hold[0,0]>=2.95, 'insufficient stopped hold')
    require(np.max(hold[:,4])<.01 and np.max(np.abs(hold[:,5]))<.02, 'robot did not remain stopped')
    require(np.max(np.linalg.norm(hold[:,1:3]-hold[0,1:3],axis=1))<.03, 'stop drift exceeded 3 cm')
    require(np.all(np.abs(cmd[(cmd[:,0]>=hold_start)&(cmd[:,0]<=t1),1:])<=1e-6), 'nonzero final command')
    states = [s.get('state') for t,s in zip(data[STATUS],status) if t0<=t[0]<=t1]
    require('frame_mismatch' not in states, 'controller frame_mismatch')
    require(not any('stale_pose' in str(s) for s in states), 'controller stale/invalid pose during run')
    require(max(abs(cmd[:,1]))<=.200001 and max(abs(cmd[:,2]))<=1.000001, 'command saturation violated')
    estimated = np.column_stack([np.interp(gt[:,0],ekf[:,0],ekf[:,i]) for i in [1,2]])
    localization = np.linalg.norm(estimated-gt[:,1:3],axis=1)
    eyaw = np.interp(gt[:,0],ekf[:,0],np.unwrap(ekf[:,3]))
    heading = np.abs(np.arctan2(np.sin(eyaw-gt[:,3]),np.cos(eyaw-gt[:,3])))
    result.update(completed=not failures, failures=sorted(set(failures)), duration_s=t1-t0,
                  reference_length_m=float(length), travelled_distance_m=travelled,
                  physical_progress_m=physical_progress, final_distance_to_start_m=final_distance,
                  tracking_error=stats(tracking), localization_error=stats(localization),
                  heading_rmse_deg=float(np.degrees(np.sqrt(np.mean(heading**2)))),
                  max_command_linear_m_s=float(max(abs(cmd[:,1]))),
                  max_command_angular_rad_s=float(max(abs(cmd[:,2]))),
                  stop_duration_s=float(hold[-1,0]-hold[0,0]),
                  initial_ground_truth=gt[0,1:4].tolist(), final_ground_truth=gt[-1,1:4].tolist())
    return result, (gt,path,estimated,tracking,localization)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('bag',nargs='?'); parser.add_argument('--output',required=True)
    parser.add_argument('--provenance')
    parser.add_argument('--aggregate',nargs=3,metavar='SUMMARY')
    args=parser.parse_args()
    output=Path(args.output); output.mkdir(parents=True,exist_ok=True)
    if args.aggregate:
        runs=[json.loads(Path(p).read_text()) for p in args.aggregate]
        if not all(r.get('completed') is True for r in runs):
            parser.error('aggregation requires three valid runs')
        if len({r['bag'] for r in runs})!=3:
            parser.error('aggregation requires independent bags')
        conditions=[(r.get('initial_state'),r.get('filter_frequency_hz'),r['reference_length_m']) for r in runs]
        if not all(c==conditions[0] for c in conditions):
            parser.error('experimental conditions differ')
        metrics={}
        for name in ['tracking_error','localization_error']:
            metrics[name]={}
            for key in ['mean_m','rmse_m','max_m']:
                values=[r[name][key] for r in runs]
                metrics[name][key]={'mean':float(np.mean(values)),'sample_std':float(np.std(values,ddof=1))}
        result={'valid_runs':3,'standard_deviation':'sample (ddof=1)',
                'metrics':metrics,'runs':runs}
        (output/'aggregate_summary.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(metrics,indent=2))
        return 0
    if not args.bag:
        parser.error('bag is required without --aggregate')
    provenance=json.loads(Path(args.provenance).read_text()) if args.provenance else None
    try:
        summary, arrays=analyze(Path(args.bag),provenance)
    except Exception as exc:
        summary,arrays={'completed':False,'failures':[str(exc)],'bag':args.bag},None
    (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    if arrays is not None:
        gt,path,estimated,tracking,localization=arrays
        np.savetxt(output/'samples.csv',np.column_stack((gt,estimated,tracking,localization)),delimiter=',',
                   header='time,x_gt,y_gt,yaw_gt,speed_gt,omega_gt,x_ekf,y_ekf,tracking_error,localization_error')
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,ax=plt.subplots(); ax.plot(*path.T,'--',label='Reference'); ax.plot(*gt[:,1:3].T,label='Ground truth')
        ax.plot(*estimated.T,label='VLP + EKF'); ax.axis('equal'); ax.legend(); ax.set(xlabel='x [m]',ylabel='y [m]')
        fig.savefig(output/'trajectory.png',dpi=180); plt.close(fig)
        for name,values in [('tracking_error',tracking),('localization_error',localization)]:
            fig,ax=plt.subplots(); ax.plot(gt[:,0]-gt[0,0],values); ax.set(xlabel='Time [s]',ylabel='Error [m]')
            fig.savefig(output/(name+'.png'),dpi=180); plt.close(fig)
    print(json.dumps(summary,indent=2))
    return 0 if summary['completed'] else 2


if __name__=='__main__':
    raise SystemExit(main())

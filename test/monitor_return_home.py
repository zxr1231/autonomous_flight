#!/usr/bin/env python3
"""Read-only return-home validation. Write CSV and JSON; never command the UAV."""
import argparse
import csv
import json
import math
import os
import threading
import time

import rospy
import rosnode
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from rospy.msg import AnyMsg

parser = argparse.ArgumentParser()
parser.add_argument('--output', required=True, help='New directory; refuses overwrite')
parser.add_argument('--timeout', type=float, default=600, help='wall seconds')
args = parser.parse_args()
os.makedirs(args.output, exist_ok=False)
rospy.init_node('return_home_verification', anonymous=True)
lock = threading.Lock()
data = {'state': 'UNKNOWN', 'xyz': None, 'home': None, 'map_points': 0,
        'odom_count': 0, 'cmd_count': 0, 'spline_count': 0, 'return_path_count': 0}
events = []

def callback(msg, key):
    with lock:
        if key == 'odom':
            p = msg.pose.pose.position
            data['xyz'] = [p.x, p.y, p.z]
            data['odom_count'] += 1
        elif key == 'home':
            p = msg.pose.position
            data['home'] = [p.x, p.y, p.z]
        elif key == 'state':
            data['state'] = msg.data
            events.append({'sim_time': rospy.get_time(), 'state': msg.data})
        elif key == 'map':
            data['map_points'] = msg.width * msg.height
        else:
            data[key + '_count'] += 1

subscriptions = []
for topic, kind, key in [
    ('/CERLAB/quadcopter/odom', Odometry, 'odom'),
    ('/dynamicExploration/home', PoseStamped, 'home'),
    ('/dynamicExploration/mission_state', String, 'state'),
    ('/dynamic_map/explored_voxel_map', PointCloud2, 'map'),
    ('/dynamicExploration/bspline_trajectory', Path, 'spline'),
    ('/dynamicExploration/return_path', Path, 'return_path'),
    ('/CERLAB/quadcopter/cmd_acc', AnyMsg, 'cmd'),
]:
    subscriptions.append(rospy.Subscriber(topic, kind, callback, callback_args=key, queue_size=1))

start = time.monotonic()
home_since = None
max_distance = 0.0
hover_errors = []
first_map_points = None
with open(os.path.join(args.output, 'samples.csv'), 'x', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['wall_elapsed', 'sim_time', 'state', 'x', 'y', 'z', 'home_distance',
                     'map_points', 'odom_count', 'cmd_count', 'spline_count'])
    while not rospy.is_shutdown() and time.monotonic() - start < args.timeout:
        time.sleep(1)
        with lock:
            sample = dict(data)
        if sample['xyz'] is None or sample['home'] is None:
            continue
        distance = math.sqrt(sum((a-b)**2 for a,b in zip(sample['xyz'], sample['home'])))
        max_distance = max(max_distance, distance)
        if first_map_points is None and sample['map_points']:
            first_map_points = sample['map_points']
        elapsed = time.monotonic() - start
        writer.writerow([elapsed, rospy.get_time(), sample['state'], *sample['xyz'], distance,
                         sample['map_points'], sample['odom_count'], sample['cmd_count'], sample['spline_count']])
        file.flush()
        if int(elapsed) % 10 == 0:
            print(json.dumps({'wall_s': round(elapsed), 'distance_home': round(distance,3), **sample}), flush=True)
        if sample['state'] == 'HOME_REACHED':
            if home_since is None:
                home_since = time.monotonic()
            hover_errors.append(distance)
            if time.monotonic() - home_since >= 10:
                break

with lock:
    result = dict(data)
    result['events'] = list(events)
result['max_distance_from_home'] = max_distance
result['max_hover_error'] = max(hover_errors) if hover_errors else None
result['first_map_points'] = first_map_points
result['wall_elapsed'] = time.monotonic() - start
result['record_rosbag'] = rospy.get_param('/autonomous_flight/record_rosbag', None)
nodes = rosnode.get_node_names()
result['unexpected_nodes'] = [n for n in nodes if n in ['/keyboard_control','/exploration_bag_info']]
result['health'] = {n: rosnode.rosnode_ping(n, max_count=1, verbose=False) for n in
                    ['/dynamic_exploration_node','/tracking_controller_node','/gazebo','/rviz']}
states = [e['state'] for e in result['events']]
result['passed'] = (result['state'] == 'HOME_REACHED' and home_since is not None and
                    time.monotonic() - home_since >= 10 and max_distance > 0.5 and
                    max(hover_errors) < 0.25 and
                    'CONFIRMING_COMPLETE' in states and 'RETURNING_HOME' in states and
                    result['return_path_count'] > 0 and result['spline_count'] > 0 and
                    result['odom_count'] > 0 and all(result['health'].values()) and
                    result['record_rosbag'] is False and not result['unexpected_nodes'])
with open(os.path.join(args.output, 'summary.json'), 'x') as file:
    json.dump(result, file, indent=2)
print(json.dumps(result, indent=2), flush=True)
raise SystemExit(0 if result['passed'] else 1)

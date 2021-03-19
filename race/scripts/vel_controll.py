#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped
import numpy as np
import dynamic_reconfigure.client
import math
from std_msgs.msg import Float64
import os
import json
from threading import Thread

lookahead = 0.634  #0.615  #0.625
max_vel = 1.68  #1.6
min_vel = 1   #1
k = 0.052   #0.23   #0.022
points_in_cal = 95  #80
slow_down_rate = 0.8  #0.71  #72
slow_down_time = 10  #10
stop = False

location = {'x': 0, 'y': 0}
end = {}


def callback(config):
    print(config['max_vel_x'])


def np_move_avg(a, n, mode="valid"):
    return np.convolve(a, np.ones((n,))/n, mode=mode)


def get_end_pose():
    global end
    path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__)))),
                        'start_game/pose.json')
    end = json.load(open(path, 'r'))['position']


rospy.init_node('vel_controller')
client = dynamic_reconfigure.client.Client("/move_base/TebLocalPlannerROS", timeout=30, config_callback=callback)
pub = rospy.Publisher('/dyaw', Float64, queue_size=20)
pub2 = rospy.Publisher('/nowG', Float64, queue_size=20)
pub3 = rospy.Publisher('/vel', Float64, queue_size=20)
q = 1 / np.linspace(1, 1 / lookahead, 300)
points = []
vels = []
globalP = 0
length = 100
dyawG = []
nowG = 0


def global_path_callback(data):
    global globalP, length, dyawG, nowG
    length = len(data.poses)
    if len(data.poses) > 100:
        data.poses = data.poses[:100]
    if len(data.poses) > 5:
        x = np_move_avg([it.pose.position.x for it in data.poses], 4)
        y = np_move_avg([it.pose.position.y for it in data.poses], 4)
        dx = x[1:] - x[:-1]
        dy = y[1:] - y[:-1]
        yaw = np.insert(np.arctan2(dx, dy), 0, location['yaw'])
        q2 = q[:len(yaw) - 1]
        dyaw = np.abs(np.diff(yaw) * q2)
        msg = Float64()
        dyawG = math.fabs(np.ptp(yaw[:points_in_cal]))
        msg.data = dyawG
        pub.publish(msg)
        if nowG > 0:
            nowG -= 1
        res = np.mean(dyaw)
        points.append(res)
        if len(points) > 3:
            points.pop(0)
        res = np.mean(points)
        globalP = res
        # print(res)


def local_path_callback(data):
    global nowG
    if len(data.poses) > 100:
        data.poses = data.poses[:100]
    if len(data.poses) > 1:
        w = np.array([it.orientation.w for it in data.poses])
        x = np.array([it.orientation.x for it in data.poses])
        y = np.array([it.orientation.y for it in data.poses])
        z = np.array([it.orientation.z for it in data.poses])
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
        dyaw = np.abs(np.diff(yaw))
        q2 = q[:len(dyaw)]
        res = np.mean(dyaw * q2)
        r = res / 4 + globalP
        vel = k / r + min_vel
        if vel > max_vel:
            vel = max_vel
        vels.append(vel)
        if len(vels) > 5:
            vels.pop(0)
        vel = np.mean(vels)
        # print(vel)
        if length < 35:
            vel *= 0.3
        if length < 25:
            vel *= 0.15
        if length < 10:
            vel *= 0.05
        if dyawG > 2:
            nowG = slow_down_time
        msg = Float64()
        msg.data = nowG / 3
        pub2.publish(msg)
        if nowG > 0:
            vel *= slow_down_rate
            print('slow down', nowG)
        if stop:
            vel = 0
            print('stop')
        msg = Float64()
        msg.data = vel
        pub3.publish(msg)
        client.update_configuration({'max_vel_x': vel})


def amcl_callback(data):
    global location
    pose = data.pose.pose.position
    ob = data.pose.pose.orientation
    w = ob.w
    x = ob.x
    y = ob.y
    z = ob.z
    location['x'] = pose.x
    location['y'] = pose.y
    location['yaw'] = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))


def stop_car():
    global stop
    rate = rospy.Rate(15)
    while not rospy.is_shutdown():
        try:
            if (location['x'] - end['x']) ** 2 + (location['y'] - end['y']) ** 2 < 1.2:
                client.update_configuration({'max_vel_x': 0})
                print('stop')
                #stop = True
            else:
                stop = False
            rate.sleep()
        except:
            print('Error happen when send vel')


if __name__ == '__main__':
    get_end_pose()
    print(end)
    receive_path = rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, global_path_callback)
    receive_path2 = rospy.Subscriber('/move_base/TebLocalPlannerROS/teb_poses', PoseArray, local_path_callback)
    receive_path3 = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_callback)
    #stop_thread = Thread(daemon=True, target=stop_car).start()
    print('ok')
    rospy.spin()



#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseArray
import numpy as np
import dynamic_reconfigure.client

max_vel = 1.9
min_vel = 0.7585
k = 0.01585
lookahead = 0.615


def callback(config):
    print(config['max_vel_x'])


rospy.init_node('vel_controller')
client = dynamic_reconfigure.client.Client("/move_base/TebLocalPlannerROS", timeout=30, config_callback=callback)
q = 1 / np.linspace(1, 1 / lookahead, 300)
points = []
vels = []
globalP = 0
length = 100


def global_path_callback(data):
    global globalP, length
    length = len(data.poses)
    if len(data.poses) > 100:
        data.poses = data.poses[:100]
    if len(data.poses) > 5:
        q2 = q[:len(data.poses) - 2]
        x = np.array([it.pose.position.x for it in data.poses])
        y = np.array([it.pose.position.y for it in data.poses])
        dx = x[1:] - x[:-1]
        dy = y[1:] - y[:-1]
        yaw = np.arctan2(dx, dy)
        dyaw = np.abs(np.diff(yaw) * q2)
        res = np.mean(dyaw)
        points.append(res)
        if len(points) > 3:
            points.pop(0)
        res = np.mean(points)
        globalP = res
        # print(res)


def local_path_callback(data):
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
        print(vel)
        if length < 30:
            vel *= 0.3
        if length < 20:
            vel *= 0.1
        if length < 10:
            vel *= 0.05
        client.update_configuration({'max_vel_x': vel})


if __name__ == '__main__':
    receive_path = rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, global_path_callback)
    receive_path2 = rospy.Subscriber('/move_base/TebLocalPlannerROS/teb_poses', PoseArray, local_path_callback)
    print('ok')
    rospy.spin()



#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseArray
import numpy as np
import dynamic_reconfigure.client

max_vel = 1.8
min_vel = 0.7
k = 0.01


def callback(config):
    print(config['max_vel_x'])


rospy.init_node('vel_controller')
client = dynamic_reconfigure.client.Client("/move_base/TebLocalPlannerROS", timeout=30, config_callback=callback)
q = 1 / np.linspace(1, 5, 80)
points = []
vels = []
globalP = 0


def global_path_callback(data):
    global globalP
    if len(data.poses) > 80:
        data.poses = data.poses[:80]
        rate = 1
    else:
        rate = len(data.poses) / 80
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
        res = np.mean(points) * rate
        globalP = res
        # print(res)


def local_path_callback(data):
    if len(data.poses) > 80:
        data.poses = data.poses[:80]
    if len(data.poses) > 1:
        w = np.array([it.orientation.w for it in data.poses])
        x = np.array([it.orientation.x for it in data.poses])
        y = np.array([it.orientation.y for it in data.poses])
        z = np.array([it.orientation.z for it in data.poses])
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
        dyaw = np.abs(np.diff(yaw))
        q2 = q[:len(dyaw)]
        res = np.mean(dyaw * q2)
        r = res / 5 + globalP
        vel = k / r + min_vel
        if vel > max_vel:
            vel = max_vel
        vels.append(vel)
        if len(vels) > 6:
            vels.pop(0)
        vel = np.mean(vels)
        print(vel)
        client.update_configuration({'max_vel_x': vel})


if __name__ == '__main__':
    receive_path = rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, global_path_callback)
    receive_path2 = rospy.Subscriber('/move_base/TebLocalPlannerROS/teb_poses', PoseArray, local_path_callback)
    print('ok')
    rospy.spin()



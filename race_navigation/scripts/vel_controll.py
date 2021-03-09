#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
import numpy as np
import dynamic_reconfigure.client


def callback(config):
    print(config['max_vel_x'])


rospy.init_node('vel_controller')
client = dynamic_reconfigure.client.Client("/move_base/TebLocalPlannerROS", timeout=30, config_callback=callback)


def path_callback(data):
    if len(data.poses) > 80:
        data.poses = data.poses[:80]
        rate = 1
    else:
        rate = len(data.poses) / 80
    x = np.array([it.pose.position.x for it in data.poses])
    y = np.array([it.pose.position.y for it in data.poses])
    dx = x[1:] - x[:-1]
    dy = y[1:] - y[:-1]
    yaw = np.arctan2(dx, dy)
    if len(yaw) > 30:
        p = np.ptp(yaw[:30])
    else:
        p = 0
    d = np.var(yaw + p / 10)
    vel = 0.05 / d * rate + 0.7
    if vel > 1.8:
        vel = 1.8
    #print(p)
    client.update_configuration({'max_vel_x': vel})


if __name__ == '__main__':
    receive_path = rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, path_callback)
    print('ok')
    rospy.spin()



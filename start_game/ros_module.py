#! /usr/bin/env python2
# -*- coding: utf-8 -*-


import time
import json
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class ROSNavNode(object):

    def __init__(self):
        rospy.init_node('test')
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        # 一定要有这一行，读 actionlib 源码可以看到 client 和 server 会在建立连接时进行协商，然后丢掉第一个 goal
        # http://docs.ros.org/en/jade/api/actionlib/html/action__client_8py_source.html
        self.client.wait_for_server()
        
        self._get_pose()
    
    def _get_pose(self):
        '''
        从文件中读取目标点位置
        '''
        with open('pose.json', 'r') as f:
            text = json.loads(f.read())
            self.pos_x = text['position']['x']
            self.pos_y = text['position']['y']
            self.pos_z = text['position']['z']
            self.ori_x = text['orientation']['x']
            self.ori_y = text['orientation']['y']
            self.ori_z = text['orientation']['z']
            self.ori_w = text['orientation']['w']
        

    def _goal_pose(self):
        '''
        构造 goal
        '''
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = self.pos_x
        self.goal.target_pose.pose.position.y = self.pos_y
        self.goal.target_pose.pose.position.z = self.pos_z
        self.goal.target_pose.pose.orientation.x = self.ori_x
        self.goal.target_pose.pose.orientation.y = self.ori_y
        self.goal.target_pose.pose.orientation.z = self.ori_z
        self.goal.target_pose.pose.orientation.w = self.ori_w
    
    def send_goal(self):
        '''
        发送 goal
        '''
        self._goal_pose()
        self.client.send_goal(self.goal)

    def get_state(self):
        '''
        判断目标完成情况，若完成目标则返回 True
        '''
        # self.client.get_state()
        # 这里其实调用 actionlib 库提供的 get_state() 方法效率更高，但是返回的貌似是 int 
        # 我懒得读源码里 Magic Number 具体代表哪一种状态了，所以直接调用下面返回字符串的方法
        if self.client.get_goal_status_text() == 'Goal reached.':
            return True
        else:
            return False

    def get_topic(self):
        '''
        获取 Topic list，返回参数为 list
        '''
        return rospy.get_published_topics()


def main():
    n = ROSNavNode()

    print('send goal')
    n.send_goal()
    n.get_state()

    time.sleep(20)

if __name__  == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\n')
        print('操作已取消')
        exit(0)
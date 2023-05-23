#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
import actionlib
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point, Pose, Quaternion
from math import pi
import os

class SendCar2Somewhere(object):
    def __init__(self):
        rospy.init_node("single_ticket")
        rospy.on_shutdown(self.SendCar2Somewhere_shutdown)

        self.move_base_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction
        )

        rospy.loginfo("等待连接move_base服务器")
        self.move_base_client.wait_for_server()
        rospy.loginfo("连上move_base 服务器了")

        self.status = 1  # 状态机状态

        quaternions = list()
        euler_angles = (
            pi / 2,
             0,
        )
        # 将上面的Euler angles转换成Quaternion的格式
        for angle in euler_angles:
            q_angle = quaternion_from_euler(0, 0, angle, axes="sxyz")
            q = Quaternion(*q_angle)
            quaternions.append(q)
        # 创建特殊点列表
        point_special = list()
        point_special.append(Pose(Point(0.885, 2.6, 0), quaternions[0]))  # 第一运动点
        point_special.append(Pose(Point(-0.4,3.7, 0), quaternions[1]))    # 手写数字终点
        
        rospy.loginfo("初始化结束")

        while not rospy.is_shutdown():
            if self.status == 1:
                rospy.loginfo("前往配药区")
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = point_special[0]
                if self.actuator_move(goal) == True:
                    self.status = 2
                else:
                    rospy.loginfo("前往第一运动点失败")
                    self.move_base_client.cancel_goal()     #取消当前目标导航点
                    self.status = 3

            elif self.status == 2:
                rospy.loginfo("前往第一运动点成功")
                self.status = 4 

            elif self.status == 4:
                rospy.loginfo("前往手写数字识别区")
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = point_special[1]
                if self.actuator_move(goal) == True:
                    self.status = 5
                else:
                    rospy.loginfo("手写数字点失败")
                    self.move_base_client.cancel_goal()     #取消当前目标导航点
                    self.status = 6

            elif self.status == 5:
                rospy.loginfo("前往手写数字点成功")
                rospy.loginfo("开始清理")
                self.move_base_client.unregister()
                os.system("rosnode kill /amcl")
                os.system("rosnode kill /base_control")
                os.system("rosnode kill /base_to_camera")
                os.system("rosnode kill /base_to_gyro")
                os.system("rosnode kill /base_to_laser")
                os.system("rosnode kill /base_to_link")
                os.system("rosnode kill /ekf_se")
                os.system("rosnode kill /joint_state_publisher")
                os.system("rosnode kill /laser_filter")
                os.system("rosnode kill /ls01d")
                os.system("rosnode kill /map_server")
                os.system("rosnode kill /joint_state_publisher")
                os.system("rosnode kill /laser_filter")
                os.system("rosnode kill /move_base")
                os.system("rosnode kill /robot_state_publisher")
                os.system("rosnode kill /single_ticket")
                rospy.loginfo("全部节点已经清理")
                exit()
            else:
                #处理失败
                self.move_base_client.cancel_goal()     #取消导航
                self.status=self.status-2 #重新发布


    # 程序退出执行
    def SendCar2Somewhere_shutdown(self):
        rospy.loginfo("Stop the robot")
        self.move_base_client.cancel_goal()     #取消当前目标导航点

if __name__ == "__main__":
    try:
       SendCar2Somewhere()
    except rospy.ROSInterruptException:
        rospy.loginfo("程序意外退出")
        pass
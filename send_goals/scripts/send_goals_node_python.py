#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 动作通信：该例程在任一仿真环境下，执行/action_client通信，消息类型move_base_msgs/MoveBaseAction MoveBaseGoal

import roslib
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from math import pi

def client():
    # 1、订阅move_base服务器的消息
    rospy.init_node("simple_goal", anonymous=True)
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    move_base.wait_for_server(rospy.Duration(5.0))
    rospy.loginfo("Connected to move base server")

    test_number = int(sys.argv[1])  # 设定编号

    quaternions = list()
    euler_angles = (
            pi / 2,
            pi / 2,
            pi / 2,
            -pi / 2,
            -pi / 2,
            -pi / 2,
            -pi / 2,
            0,
            pi,
    )
        # 将上面的Euler angles转换成Quaternion的格式
    for angle in euler_angles:
        q_angle = quaternion_from_euler(0, 0, angle, axes="sxyz")
        q = Quaternion(*q_angle)
        quaternions.append(q)
    
    point_test = list()
    point_test.append(Pose(Point(1.4, 2.3, 0), quaternions[0]))  # A点
    point_test.append(Pose(Point(1.28, 2.71, 0), quaternions[1]))  # B点
    point_test.append(Pose(Point(1.28, 1.80, 0), quaternions[2]))  # C点

    point_test.append(Pose(Point(-1.90, 2.10, 0), quaternions[3]))  # 1点
    point_test.append(Pose(Point(-1.03, 1.63, 0), quaternions[4]))  # 2点
    point_test.append(Pose(Point(-1.88, 1.13, 0), quaternions[5]))  # 3点
    point_test.append(Pose(Point(-1.03, 0.68, 0), quaternions[6]))  # 4点
    # 2、目标点内容
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = point_test[test_number]

    # 3、将目标点发送出去
    rospy.loginfo("Sending goal")
    move_base.send_goal(goal)

    # 4、五分钟时间限制 查看是否成功到达
    finished_within_time = move_base.wait_for_result(rospy.Duration(300))
    if not finished_within_time:
        move_base.cancel_goal()
        rospy.loginfo("Timed out achieving goal")
    else:
        state = move_base.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal succeeded!")
        else:
            rospy.loginfo("Goal failed！ ")


if __name__ == "__main__":
    client()

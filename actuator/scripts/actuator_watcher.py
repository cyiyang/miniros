#! /usr/bin/env python
# -*- coding: utf-8 -*-
import os
import threading
from actuator_points import point_special_watcher
import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from statemachine import State, StateMachine
from actuator.msg import EveryoneStatus


def thread_Slave():
    rospy.spin()


def Death_Rattle():
    rospy.logerr("启动Yolo")
    # 启动 Yolo
    os.system("/home/watcher/digit_recognizer/build/digit_recognizer_demo")
    exit()


class SimpleStateMachine(StateMachine):
    def __init__(self, actuator):
        self.actuator = actuator
        super(SimpleStateMachine, self).__init__()

    Start = State("Start", initial=True)
    Temporary = State("Temporary")
    Harbour = State("Harbour")
    # Go是一个事件Event，这个Event是由几个转移Transitions组成
    Single_Ticket = (
        Start.to(Temporary, cond="AllowedGo")
        | Temporary.to(Harbour)
        | Start.to(Start, cond="Wait")
    )

    def Wait(self):
        rospy.loginfo("Master尚未到达,请等待")
        rospy.sleep(1)
        return True

    def AllowedGo(self):
        #或者这里可以改成ABC
        if self.actuator.master_location == "HandWritten":
            return True
        else:
            return False

    def before_transition(self,state):
        Watcher_status = EveryoneStatus()
        Watcher_status.name = 'Watcher'
        Watcher_status.status = state.id
        self.actuator.location_pub.publish(Watcher_status)

    def on_enter_Temporary(self):
        rospy.loginfo("前往临时停靠点")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "watcher/map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = point_special_watcher[0]
        if self.actuator.SendCar2Somewhere_move(goal) == True:
            rospy.loginfo("到达临时停靠点")
        else:
            rospy.logerr("临时停靠点失败")
            self.actuator.move_base_client.cancel_goal()

    def on_enter_Harbour(self):
        rospy.logwarn("前往识别区")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "watcher/map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = point_special_watcher[1]
        if self.actuator.SendCar2Somewhere_move(goal) == True:
            rospy.logwarn("到达识别区")
            Watcher_status = EveryoneStatus()
            Watcher_status.name = 'Watcher'
            Watcher_status.status = 'Harbour'
            self.actuator.location_pub.publish(Watcher_status)
            Death_Rattle()
        else:
            rospy.logerr("识别区失败")
            self.actuator.move_base_client.cancel_goal()


class SendCar2Somewhere(object):
    def __init__(self):
        rospy.init_node("act_watcher")
        rospy.on_shutdown(self.SendCar2Somewhere_shutdown)
        self.location_sub = rospy.Subscriber(
            "/location",
            EveryoneStatus,
            self.SendCar2Somewhere_deallocation,
            queue_size=10,
        )
        self.location_pub = rospy.Publisher(
            "/location",
            EveryoneStatus,
            queue_size=10,
        )

        self.master_location = "Start"  # 从车一开始默认为Start

        self.move_base_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction
        )
        rospy.loginfo("等待连接move_base服务器")
        self.move_base_client.wait_for_server()
        rospy.logwarn("连上move_base 服务器了")

        add_thread = threading.Thread(target=thread_Slave)
        add_thread.start()
        rospy.loginfo("deal Master thread OK")

    def SendCar2Somewhere_shutdown(self):
        rospy.logerr("Stop the robot")
        self.move_base_client.cancel_goal()  # 取消当前目标导航点

    def SendCar2Somewhere_move(self, goal):
        self.move_base_client.send_goal(goal)
        finished_within_time = self.move_base_client.wait_for_result(rospy.Duration(30))
        if not finished_within_time:
            self.move_base_client.cancel_goal()
            rospy.logerr("move_base超时")
        else:
            state = self.move_base_client.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("成功到达")
                return True
            else:
                rospy.logerr("没超时但失败")
                return False

    def SendCar2Somewhere_deallocation(self, msg):
        if msg.name == "Master":
            self.master_location = msg.status


if __name__ == "__main__":
    try:
        Watcher_Car = SendCar2Somewhere()
        machine = SimpleStateMachine(Watcher_Car)
        while not rospy.is_shutdown():
            machine.Single_Ticket()
    except rospy.ROSInterruptException:
        rospy.logerr("程序意外退出")
        pass

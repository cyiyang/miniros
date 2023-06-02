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
    rospy.loginfo("开始清理")
    os.system("rosnode kill amcl")
    os.system("rosnode kill base_control")
    os.system("rosnode kill base_to_camera")
    os.system("rosnode kill base_to_gyro")
    os.system("rosnode kill base_to_laser")
    os.system("rosnode kill base_to_link")
    os.system("rosnode kill ekf_se")
    os.system("rosnode kill joint_state_publisher")
    os.system("rosnode kill laser_filter")
    os.system("rosnode kill ls01d")
    os.system("rosnode kill map_server")
    os.system("rosnode kill joint_state_publisher")
    os.system("rosnode kill laser_filter")
    os.system("rosnode kill move_base")
    os.system("rosnode kill robot_state_publisher")
    os.system("rosnode kill single_ticket")
    rospy.loginfo("全部节点已经清理,开始亡语")
    # 启动 Yolo
    # path = os.path.expanduser(
    #     "~/drug-deliverer/drug-deliverer/digit_recognizer/build"
    # )
    # os.chdir(path)
    # os.system("./digit_recognizer_demo")
    exit()


class SimpleStateMachine(StateMachine):
    def __init__(self, actuator):
        self.actuator = actuator
        super(SimpleStateMachine, self).__init__()

    Start = State("Start", initial=True)
    Temporary =  State("Temporary")
    Harbour  =  State("Harbour")
    # Go是一个事件Event，这个Event是由几个转移Transitions组成
    Single_Ticket = (
        Start.to(Temporary, cond="AllowedGo")|
        Temporary.to(Harbour)
    )

    def AllowedGo(self):
        if(self.actuator.slave_location == 'Dispense_ABC'):
            return True
        else:
            return False
        
def on_transition(self,state):
    rospy.loginfo("Watcher想要转移,状态为:%s",state.id)
    if state.id == 'Start': #在Start时进入转移，代表条件已经允许，可以直接pass
        pass
    else:
        while self.actuator.slave_location == state.id :
            rospy.loginfo("主从状态一致，请等待")


def on_enter_Temporary(self):
    rospy.loginfo("前往临时停靠点")
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "watcher/map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = point_special_watcher[0]
    if self.actuator.actuator_move(goal) == True:
        rospy.loginfo("到达临时停靠点")
    else:
        rospy.logerr("配药失败")
        self.actuator.move_base_client.cancel_goal()
def on_enter_Harbour(self):
    rospy.loginfo("前往识别区")
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "watcher/map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = point_special_watcher[1]
    if self.actuator.actuator_move(goal) == True:
        rospy.loginfo("到达识别区")
        rospy.sleep(2)
        Death_Rattle()
    else:
        rospy.logerr("配药失败")
        self.actuator.move_base_client.cancel_goal()


class SendCar2Somewhere(object):
    def __init__(self):
        rospy.init_node("single_ticket")
        rospy.on_shutdown(self.SendCar2Somewhere_shutdown)
        self.location_sub = rospy.Subscriber("/location",EveryoneStatus,self.SendCar2Somewhere_deallocation,queue_size=10)

        self.slave_location = 'Start' #从车一开始默认为Start

        self.move_base_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction
        )
        rospy.loginfo("等待连接move_base服务器")
        self.move_base_client.wait_for_server()
        rospy.loginfo("连上move_base 服务器了")

        add_thread = threading.Thread(target=thread_Slave)
        add_thread.start()
        rospy.loginfo("deal Master thread OK")
        

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
    
    def SendCar2Somewhere_deallocation(self,msg):
        rospy.loginfo("接收状态为:姓名:%s,状态:%s",msg.name,msg.status)
        if msg.name == 'Slave':
            self.slave_location=msg.status
        pass



if __name__ == "__main__":
    try:
        Watcher_Car = SendCar2Somewhere()
        machine = SimpleStateMachine(Watcher_Car)
        while not rospy.is_shutdown():
            machine.Single_Ticket()
    except rospy.ROSInterruptException:
        rospy.logerr("程序意外退出")
        pass
#! /usr/bin/env python
# -*- coding: utf-8 -*-

import threading
from actuator_points import point_ABC_slave, point_1234_slave, point_special_slave
import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from playsound import playsound
from statemachine import State, StateMachine
from actuator.srv import (
    DestinationMsg,
    DestinationMsgRequest,
)
from actuator.msg import EveryoneStatus

def thread_Master():
    rospy.spin()

class SimpleStateMachine(StateMachine):
    def __init__(self, actuator):
        self.actuator = actuator
        super(SimpleStateMachine, self).__init__()

    Start = State("Start", initial=True)
    Dispense_ABC = State("Dispense_ABC")
    HandWritten = State("HandWritten")
    Pickup_1234 = State("Pickup_1234")
    Zero = State("Zero")
    Wander1 = State("Wander1")
    Wander2 = State("Wander2")
    Rubbish = State("Rubbish")
    # Go是一个事件Event，这个Event是由几个转移Transitions组成
    Go = (
        Start.to(Dispense_ABC, cond="GotTarget")
        | Start.to(Start, cond="ReAskMission")
        | Start.to(Wander1, cond="StartWander")
        | Dispense_ABC.to(HandWritten)
        | HandWritten.to(Rubbish,cond="GetRubbish")
        | HandWritten.to(Pickup_1234)
        | Rubbish.to(Zero)
        | Pickup_1234.to(Zero)
        | Zero.to(Start)
        | Wander1.to(Wander2)
        | Wander2.to(Zero)
    )

    def on_transition(self,state):
        Slave_status = EveryoneStatus()
        Slave_status.name = 'Slave'
        Slave_status.status = state.id
        self.actuator.location_pub.publish(Slave_status)
        rospy.logwarn("请求从[%s]转移",state.id)
        if state.id == 'Start': #在Start时进入转移，代表条件已经允许，可以直接pass
            pass
        else:
            while self.actuator.master_location == state.id :
                rospy.logwarn("主从状态一致，请等待")


    def GotTarget(self):
        if (self.actuator.asksuccess_flag):
            if self.actuator.watcher_location =='Harbour' and self.actuator.master_location == 'Dispense_ABC':
                rospy.logwarn("从车出发")
                return True
            else:
                return False
        else:
            return False



    def StartWander(self):
        if self.actuator.master_location == 'Wander1' :
            rospy.logwarn("进入wander状态")
            return True
        else:
            return False

    def ReAskMission(self):
        if self.actuator.master_location == 'Start' or (not self.GotTarget() and not self.StartWander()): #主车在Start，从车只能reask
            return True
        else:
            return False

    def GetRubbish(self):
        if (self.actuator.mission_response.deliver_destination==5):
            rospy.logwarn("丢弃")
            return True
        else:
            return False



    def on_enter_Start(self):
        self.actuator.actuator_ask_newtarget()
        rospy.sleep(1)

    def on_enter_Dispense_ABC(self):

        rospy.loginfo("前往配药区")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "slave/map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = point_ABC_slave[
            self.actuator.mission_response.drug_location
        ]
        if self.actuator.actuator_move(goal) == True:
            newABC = self.actuator.responseToABC[
                self.actuator.mission_response.drug_location
            ]
            rospy.loginfo("到达配药区%c", newABC)
            self.actuator.actuator_updateABC()
        else:
            rospy.logerr("配药失败")
            self.actuator.move_base_client.cancel_goal()

    def on_enter_HandWritten(self):
        rospy.loginfo("前往手写数字识别区")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "slave/map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = point_special_slave[1]
        if self.actuator.actuator_move(goal) == True:
            rospy.loginfo("到达手写数字点")
        else:
            rospy.logerr("手写数字点失败")
            self.actuator.move_base_client.cancel_goal()

    def on_enter_Pickup_1234(self):
        rospy.loginfo("前往取药区")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "slave/map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = point_1234_slave[
            self.actuator.mission_response.deliver_destination
        ]
        if self.actuator.actuator_move(goal) == True:
            rospy.loginfo("到达取药区%d", self.actuator.mission_response.deliver_destination)
            self.actuator.actuator_update1234()

        else:
            rospy.logerr("取药失败")
            self.actuator.move_base_client.cancel_goal()  # 取消当前目标导航点

    def on_enter_Rubbish(self):
        rospy.logwarn("前往垃圾桶")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "slave/map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose =  point_special_slave[3]
        if self.actuator.actuator_move(goal) == True:
            rospy.loginfo("到达垃圾桶")
            self.actuator.actuator_throwRubbish()
        else:
            rospy.logerr("丢垃圾失败！")
            self.actuator.move_base_client.cancel_goal()  # 取消当前目标导航点

    def on_enter_Zero(self):
        rospy.loginfo("前往起点")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "slave/map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = point_special_slave[0]
        if self.actuator.actuator_move(goal) == True:
            rospy.loginfo("到达起点")
        else:
            rospy.logerr("前往起点失败")
            self.actuator.move_base_client.cancel_goal()  # 取消当前目标导航点

    def on_enter_Wander1(self):
        rospy.loginfo("前往运动点（配药区）")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "slave/map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = point_special_slave[2]
        if self.actuator.actuator_move(goal) == True:
            rospy.loginfo("到达运动点（配药区）")
        else:
            rospy.logerr("前往1号运动点失败")
            self.actuator.move_base_client.cancel_goal()  # 取消当前目标导航点

    def on_enter_Wander2(self):

        rospy.loginfo("前往手写数字识别区")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "slave/map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = point_special_slave[1]
        if self.actuator.actuator_move(goal) == True:
            rospy.loginfo("到达手写数字点")
        else:
            rospy.logerr("手写数字点失败")
            self.actuator.move_base_client.cancel_goal()

        rospy.loginfo("前往运动点（取药区）")
        goal2 = MoveBaseGoal()
        goal2.target_pose.header.frame_id = "slave/map"
        goal2.target_pose.header.stamp = rospy.Time.now()
        goal2.target_pose.pose = point_special_slave[3]
        if self.actuator.actuator_move(goal2) == True:
            rospy.loginfo("到达运动点（配药区）")

        else:
            rospy.logerr("前往2号运动点失败")
            self.actuator.move_base_client.cancel_goal()  # 取消当前目标导航点

class CarActuator(object):
    def __init__(self):
        rospy.init_node("act_slave")
        self.location_sub = rospy.Subscriber("/location",EveryoneStatus,self.actuator_deallocation,queue_size=10)
        self.location_pub = rospy.Publisher("/location",EveryoneStatus,queue_size=10)
        self.asksuccess_flag = False  # 请求成功标志位
        self.master_location = 'Start' #主车一开始默认为Start
        self.watcher_location = 'Start' #主车一开始默认为Start

        rospy.on_shutdown(self.actuator_shutdown)

        # 订阅move_base服务器的消息
        self.move_base_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction
        )
        rospy.loginfo("等待连接move_base服务器")
        self.move_base_client.wait_for_server()
        rospy.logwarn("连上move_base 服务器了")

        self.mission_client = rospy.ServiceProxy("/mission", DestinationMsg)
        rospy.loginfo("调度器客户端正常启动了")

        self.mission_client.wait_for_service()
        rospy.loginfo("连上调度器服务器了")

        self.mission_request = DestinationMsgRequest()  # 定义请求

        self.responseToABC = {-1: "E", 0: "A", 1: "B", 2: "C"}

        add_thread = threading.Thread(target=thread_Master)
        add_thread.start()
        rospy.loginfo("deal Master thread OK")

 # 程序退出执行
    def actuator_shutdown(self):
        rospy.logerr("Stop the robot")
        self.move_base_client.cancel_goal()  # 取消当前目标导航点

    # 向服务器请求新任务,到达起点
    def actuator_ask_newtarget(self):
        self.mission_request.request_type = 1  # 请求包编号为“请求新任务”
        self.mission_response = self.mission_client.call(
            1, self.mission_request.request_type, 0, 0
        )
        rospy.loginfo("代号:%d,请求新任务",1)
        rospy.loginfo(
            "Get:%c,Send:%d",
            self.responseToABC[self.mission_response.drug_location],
            self.mission_response.deliver_destination,
        )
        if (
            self.mission_response.drug_location != -1
            and self.mission_response.deliver_destination != -1
        ):  # 不是负-1代表请求成功
            self.asksuccess_flag = True  # 请求成功
        else:  # 请求失败
            self.asksuccess_flag = False

    # 向服务器上报已取药
    def actuator_updateABC(self):
        playsound("/home/slave/Music/dispense.mp3")
        self.mission_request.request_type = 2  # 请求包编号为“完成配药/ABC”
        self.mission_client.call(1, self.mission_request.request_type, 0, 0)

    # 向服务器上报已送药
    def actuator_update1234(self):
        playsound("/home/slave/Music/pick_up.mp3")
        self.mission_request.request_type = 3  # 请求包编号为“完成送药/1234”
        self.mission_client.call(1, self.mission_request.request_type, 0, 0)

    def actuator_throwRubbish(self):
        playsound("/home/slave/Music/throwRubbish.mp3")
        self.mission_request.request_type = 3  # 请求包编号为“完成送药/1234”
        self.mission_client.call(1, self.mission_request.request_type, 0, 0)

    def actuator_move(self, goal):
        self.move_base_client.send_goal(goal)
        finished_within_time = self.move_base_client.wait_for_result(rospy.Duration(15))
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

    def actuator_deallocation(self,msg):
        if msg.name == 'Master':
            self.master_location=msg.status
        elif msg.name == 'Watcher' :
            self.watcher_location=msg.status



if __name__ == "__main__":
    try:
        Slave_Car = CarActuator()
        machine = SimpleStateMachine(Slave_Car)
        while not rospy.is_shutdown():
            machine.Go()
    except rospy.ROSInterruptException:
        rospy.logerr("程序意外退出")
        pass
#! /usr/bin/env python
# -*- coding: utf-8 -*-

import threading
from actuator_points import point_ABC_slave, point_1234_slave, point_special_slave
import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from playsound import playsound
from std_msgs.msg import String
from statemachine import State, StateMachine
from actuator.srv import (
    DestinationMsg,
    DestinationMsgRequest,
)

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
    Wandering = State("Wandering")
    # Go是一个事件Event，这个Event是由几个转移Transitions组成
    Go = (
        Start.to(Dispense_ABC, cond="GotTarget")
        | Start.to(Start, cond="ReAskMission")
        | Start.to(Wandering, cond="StartWander")
        | Dispense_ABC.to(HandWritten)
        | HandWritten.to(Pickup_1234)
        | Pickup_1234.to(Zero)
        | Zero.to(Start)
        | Wandering.to(Zero)
    )

    def before_transition(self, event, state):
        #判断主车状态，条件满足即可转移，否则卡在这里
        pass

    def GotTarget(self):
        pass

    def ReAskMission(self):
        pass

    def StartWander(self):
        pass




class CarActuator(object):
    def __init__(self):
        rospy.init_node("act_slave")
        self.master_location_sub = rospy.Subscriber("master_location",String,self.actuator_dealmaster,queue_size=10)

        self.first_arrived_flag = False  # 第一次到达标志位
        self.asksuccess_flag = False  # 请求成功标志位

        rospy.on_shutdown(self.actuator_shutdown)

        # 订阅move_base服务器的消息
        self.move_base_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction
        )
        rospy.loginfo("等待连接move_base服务器")
        self.move_base_client.wait_for_server()
        rospy.loginfo("连上move_base 服务器了")

        self.mission_client = rospy.ServiceProxy("mission", DestinationMsg)
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
        rospy.loginfo("车辆代号:%d,请求新任务",1)
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
        playsound("/home/EPRobot/Music/dispense.mp3")
        self.mission_request.request_type = 2  # 请求包编号为“完成配药/ABC”
        self.mission_client.call(1, self.mission_request.request_type, 0, 0)

    # 向服务器上报已送药
    def actuator_update1234(self):
        playsound("/home/EPRobot/Music/pick_up.mp3")
        self.mission_request.request_type = 3  # 请求包编号为“完成送药/1234”
        self.mission_client.call(1, self.mission_request.request_type, 0, 0)

    def actuator_move(self, goal):
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
    
    def actuator_dealMaster(self,msg):
        rospy.loginfo("接收到Master的状态为:%s",msg.data)
        pass



if __name__ == "__main__":
    try:
        Slave_Car = CarActuator()
        machine = SimpleStateMachine(Slave_Car)
        while not rospy.is_shutdown():
            machine.Go()
    except rospy.ROSInterruptException:
        rospy.logerr("程序意外退出")
        pass
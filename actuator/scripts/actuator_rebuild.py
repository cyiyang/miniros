#! /usr/bin/env python
# -*- coding: utf-8 -*-

import threading
from actuator_points import point_ABC_master,point_1234_master,point_special_master
import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from playsound import playsound
from std_msgs.msg import Bool,Int16
from statemachine import State, StateMachine
from statemachine.exceptions import TransitionNotAllowed


from actuator.srv import (
    DestinationMsg,
    DestinationMsgRequest,
    PermissionMsg,
    PermissionMsgResponse,
)


def thread_CV():
    rospy.spin()


class SimpleStateMachine(StateMachine):
    Start = State("Start",initial=True)
    Dispense_ABC = State("Dispense_ABC")
    HandWritten = State("HandWritten")
    Pickup_1234 = State("Pickup_1234")
    Zero = State("Zero")
    Wandering = State("Wandering")
    #Go是一个事件Event，这个Event是由几个转移Transitions组成
    Go = (Start.to(Dispense_ABC,cond="GotTarget")|Start.to(Start,cond="ReAskMission")|Start.to(Wandering,cond="StartWander")|Dispense_ABC.to(HandWritten)|HandWritten.to(Pickup_1234)|Pickup_1234.to(Zero)|Zero.to(Start)|Wandering.to(Start))

    def on_enter_Start(self): 
        '''
        需要完成的工作：
        1.请求新任务
        2.允许识别器查看图片
        '''
        Master_Car.actuator_ask_newtarget()
        Master_Car.allow2see_flag=True

    def on_enter_Dispense_ABC(self):
        '''
        需要完成的工作：
        1.向Move_Base发布关于ABC的点
        2.判断是否到达
        3.如果到达请上报服务器
        4.如果没有到达，请阻塞在这里
        5.如果失败，请取消目标点
        '''
        rospy.loginfo("前往配药区")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = point_ABC_master[Master_Car.mission_response.drug_location]
        if Master_Car.actuator_move(goal) == True:
            newABC = Master_Car.responseToABC[Master_Car.mission_response.drug_location]
            rospy.loginfo("到达配药区%c", newABC)
            Master_Car.actuator_updateABC()
        else:
            rospy.logerr("配药失败")
            Master_Car.move_base_client.cancel_goal()
    
    def on_enter_HandWritten(self):
        '''
        需要完成的工作：
        1.向Move_Base发布关于手写数字的点
        2.判断是否到达
        3.如果到达请上报服务器
        4.如果没有到达，请阻塞在这里
        5.如果失败，请取消目标点
        6.对第一次到达进行处理
        '''
        rospy.loginfo("前往手写数字识别区")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = point_special_master[1]
        if Master_Car.actuator_move(goal) == True:
            rospy.loginfo("到达手写数字点")
            if(Master_Car.first_arrived_flag==False):#第一次到达标志位
                rospy.logwarn("Master已到达，Watcher可以离开")
                Master_Car.first_arrived_flag=True
                Master_Car.watcher_go_pub.publish(True)
        else:
            rospy.logerr("手写数字点失败")
            Master_Car.move_base_client.cancel_goal()
    
    def on_enter_Pickup_1234(self):
        rospy.loginfo("前往取药区")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = point_1234_master[Master_Car.mission_response.deliver_destination]
        if Master_Car.actuator_move(goal) == True:
            rospy.loginfo("到达取药区%d", Master_Car.mission_response.deliver_destination)
            Master_Car.actuator_update1234()
             
        else:
            rospy.logerr("取药失败")
            Master_Car.move_base_client.cancel_goal()  # 取消当前目标导航点
    
    def on_enter_Zero(self):
        rospy.loginfo("前往起点")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = point_special_master[0]
        if Master_Car.actuator_move(goal) == True:
           rospy.loginfo("到达起点")
        else:
            rospy.logerr("前往起点失败")
            Master_Car.move_base_client.cancel_goal()  # 取消当前目标导航点

    #以下是条件转移的条件
    def GotTarget(self):
        if Master_Car.asksuccess_flag:
            return True
        else:
            return False
    
    def ReAskMission(self):
        if not (Master_Car.seefinished_flag and Master_Car.asksuccess_flag): 
            '''
            seefinished_flag==False ->有新识别器需求的请求
            asksuccess_flag==False->请求失败
            '''
            return True
        else:
            return False
    
    def StartWander(self):
        if Master_Car.wander_flag:
            Master_Car.wander_flag = False 
            return True
        else:
            return False

class CarActuator(object):
    def __init__(self):
        rospy.init_node("act_master")
        self.watcher_go_pub = rospy.Publisher("watcher_go",Bool,queue_size=10)
        # self.master_status_pub = rospy.Publisher("master_status",Int16,queue_size=10)
        self.machine = SimpleStateMachine()

        self.first_arrived_flag = False     #第一次到达标志位
        self.asksuccess_flag = False        #请求成功标志位
        self.seefinished_flag = False       # 识别结束标志位
        self.allow2see_flag = True          #允许识别标志位
        self.wander_flag = False            #允许进入Wander标志位

        rospy.on_shutdown(self.actuator_shutdown)

        self.permission_server = rospy.Service(
            "permission", PermissionMsg, self.actuator_dealCV_ask
        )
        rospy.loginfo("命令服务器正常启动")

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

        self.status = 1  # 状态机状态
        self.mission_request = DestinationMsgRequest()  # 定义请求

        self.responseToABC = {-1: "E", 0: "A", 1: "B", 2: "C"}

        add_thread = threading.Thread(target=thread_CV)
        add_thread.start()
        rospy.loginfo("deal CV thread OK")

        while not rospy.is_shutdown():
            self.machine.Go()
          

    # 程序退出执行
    def actuator_shutdown(self):
        rospy.logerr("Stop the robot")
        self.move_base_client.cancel_goal()  # 取消当前目标导航点

    # 向服务器请求新任务,到达起点（标准数字区）
    def actuator_ask_newtarget(self):
        self.mission_request.request_type = 1  # 请求包编号为“请求新任务”
        self.mission_response = self.mission_client.call(
            0, self.mission_request.request_type, 0, 0
        )
        rospy.loginfo("车辆代号:%d,请求新任务", 0)
        rospy.loginfo(
            "Get:%c,Send:%d",
            self.responseToABC[self.mission_response.drug_location],
            self.mission_response.deliver_destination,
        )
        if (
            self.mission_response.drug_location != -1
            and self.mission_response.deliver_destination != -1
        ):  # 不是负-1代表请求成功
            self.asksuccess_flag = True #请求成功
            self.wander_flag = False    #不允许进入Wander状态
        else:  # 请求失败
            self.asksuccess_flag = False

        #如果请求失败同时已经看过了，那么就代表已经完成本轮
        if self.asksuccess_flag == False and self.seefinished_flag == True:
            self.wander_flag = True #允许进入wander状态 
            rospy.logwarn("进入wandering状态")

    # 向服务器上报已取药
    def actuator_updateABC(self):
        playsound("/home/EPRobot/Music/dispense.mp3")
        self.mission_request.request_type = 2  # 请求包编号为“完成配药/ABC”
        self.mission_client.call(
            0, self.mission_request.request_type, 0, 0
        )

    # 向服务器上报已送药
    def actuator_update1234(self):
        playsound("/home/EPRobot/Music/pick_up.mp3")
        self.mission_request.request_type = 3  # 请求包编号为“完成送药/1234”
        self.mission_client.call(
            0, self.mission_request.request_type, 0, 0
        )

    # 处理来自识别器的请求
    def actuator_dealCV_ask(self, req):
        if req.request == 0:  # "想看请求"
            rospy.loginfo("接受:[想看]")
            self.seefinished_flag = False       #未看完=想看=接受到想看请求=有新一轮
            if self.allow2see_flag:             #已经到达识别区,允许识别
                resp = PermissionMsgResponse(1)  # 可以看
                rospy.loginfo("回复:[可以]")
            else:
                resp = PermissionMsgResponse(0)  # 不可以

                rospy.loginfo("回复:[拒绝]")
        else:  # "看完了"
            rospy.loginfo("接受:[看完]")
            resp = PermissionMsgResponse(0)
            self.seefinished_flag = True  #识别结束=不需要识别

        return resp

    def actuator_move(self, goal):
        # 把目标位置发送给MoveBaseAction的服务器
        self.move_base_client.send_goal(goal)
        # 设定1分钟的时间限制
        finished_within_time = self.move_base_client.wait_for_result(rospy.Duration(30))
        # 如果30s没有成功，放弃
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


if __name__ == "__main__":
    try:
        Master_Car=CarActuator()
    except rospy.ROSInterruptException:
        rospy.logerr("程序意外退出")
        pass

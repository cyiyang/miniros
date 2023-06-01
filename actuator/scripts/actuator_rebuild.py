#! /usr/bin/env python
# -*- coding: utf-8 -*-

import threading
from actuator_points import point_ABC_master,point_1234_master,point_special_master
import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from playsound import playsound
from std_msgs.msg import Bool
from statemachine import State, StateMachine
from actuator.srv import (
    DestinationMsg,
    DestinationMsgRequest,
    PermissionMsg,
    PermissionMsgResponse,
)

def thread_CV():
    rospy.spin()

class SimpleStateMachine(StateMachine):
    def __init__(self,actuator):
        self.actuator=actuator
        super(SimpleStateMachine,self).__init__()
    Start = State("Start",initial=True)
    Dispense_ABC = State("Dispense_ABC")
    HandWritten = State("HandWritten")
    Pickup_1234 = State("Pickup_1234")
    Zero = State("Zero")
    Wandering = State("Wandering")
    #Go是一个事件Event，这个Event是由几个转移Transitions组成
    Go = (Start.to(Dispense_ABC,cond="GotTarget")|
          Start.to(Start,cond="ReAskMission")|
          Start.to(Wandering,cond="StartWander")|
          Dispense_ABC.to(HandWritten)|
          HandWritten.to(Pickup_1234)|
          Pickup_1234.to(Zero)|
          Zero.to(Start)|
          Wandering.to(Zero))

    #以下是条件转移的条件
    '''                seefinished_flag    asksuccess_flag      意义
    True or False             0                  0              得到识别请求，请求失败，重新请求
                              0                  1              得到识别请求，请求成功，出现小哥堆积情况，继续配送
                              1                  0              识别结束，请求失败，本轮小哥处理完，进入wander
                              1                  1              识别结束，请求成功，常规情况，继续配送
    '''
    def GotTarget(self):
        if self.actuator.asksuccess_flag :
            return True
        else:
            return False
    
    def ReAskMission(self):
        if not (self.actuator.seefinished_flag_true or self.actuator.asksuccess_flag): 
            rospy.sleep(1)
            return True
        else:
            return False
    
    def StartWander(self):
        if self.actuator.seefinished_flag_true == True and self.actuator.asksuccess_flag == False :
            rospy.logwarn("进入wandering状态")
            return True
        else:
            return False

        
    def on_enter_Start(self): 
        '''
        需要完成的工作：
        1.请求新任务
        2.允许识别器查看图片
        '''
        self.actuator.actuator_ask_newtarget()
        self.actuator.allow2see_flag=True
        self.actuator.seefinished_flag_true=self.actuator.seefinished_flag_temp

    def on_exit_Start(self):
        self.actuator.allow2see_flag=False
        
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
        goal.target_pose.pose = point_ABC_master[self.actuator.mission_response.drug_location]
        if self.actuator.actuator_move(goal) == True:
            newABC = self.actuator.responseToABC[self.actuator.mission_response.drug_location]
            rospy.loginfo("到达配药区%c", newABC)
            self.actuator.actuator_updateABC()
        else:
            rospy.logerr("配药失败")
            self.actuator.move_base_client.cancel_goal()
    
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
        if self.actuator.actuator_move(goal) == True:
            rospy.loginfo("到达手写数字点")
            if(self.actuator.first_arrived_flag==False):#第一次到达标志位
                rospy.logwarn("Master已到达，Watcher可以离开")
                self.actuator.first_arrived_flag=True
                self.actuator.watcher_go_pub.publish(True)
        else:
            rospy.logerr("手写数字点失败")
            self.actuator.move_base_client.cancel_goal()
    
    def on_enter_Pickup_1234(self):
        rospy.loginfo("前往取药区")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = point_1234_master[self.actuator.mission_response.deliver_destination]
        if self.actuator.actuator_move(goal) == True:
            rospy.loginfo("到达取药区%d", self.actuator.mission_response.deliver_destination)
            self.actuator.actuator_update1234()
             
        else:
            rospy.logerr("取药失败")
            self.actuator.move_base_client.cancel_goal()  # 取消当前目标导航点
    
    def on_enter_Zero(self):
        rospy.loginfo("前往起点")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = point_special_master[0]
        if self.actuator.actuator_move(goal) == True:
           rospy.loginfo("到达起点")
        else:
            rospy.logerr("前往起点失败")
            self.actuator.move_base_client.cancel_goal()  # 取消当前目标导航点

    def on_enter_Wandering(self):
        rospy.loginfo("前往运动点（配药区）")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = point_special_master[2]
        if self.actuator.actuator_move(goal) == True:
            rospy.loginfo("到达运动点（配药区）")
        else:
            rospy.logerr("前往1号运动点失败")
            self.actuator.move_base_client.cancel_goal()  # 取消当前目标导航点

        rospy.loginfo("前往运动点（取药区）")
        goal2 = MoveBaseGoal()
        goal2.target_pose.header.frame_id = "map"
        goal2.target_pose.header.stamp = rospy.Time.now()
        goal2.target_pose.pose = point_special_master[3]
        if self.actuator.actuator_move(goal2) == True:
            rospy.loginfo("到达运动点（配药区）")
        else:
            rospy.logerr("前往2号运动点失败")
            self.actuator.move_base_client.cancel_goal()  # 取消当前目标导航点

class CarActuator(object):
    def __init__(self):
        rospy.init_node("act_master")
        self.watcher_go_pub = rospy.Publisher("watcher_go",Bool,queue_size=10)

        self.first_arrived_flag = False     #第一次到达标志位
        self.asksuccess_flag = False        #请求成功标志位
        self.seefinished_flag_true = False  #识别结束标志位
        self.seefinished_flag_temp = False  #识别结束标志位影子寄存
        self.allow2see_flag = True          #允许识别标志位,物理上的
        
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
            self.seefinished_flag_temp = False       #未看完=想看=接受到想看请求=有新一轮
            if self.allow2see_flag:             #已经到达识别区,允许识别
                self.allow2see_flag=False
                resp = PermissionMsgResponse(1)  # 可以看
                rospy.loginfo("回复:[可以]")
            else:
                resp = PermissionMsgResponse(0)  # 不可以

                rospy.loginfo("回复:[拒绝]")
        else:  # "看完了"
            rospy.loginfo("接受:[看完]")
            resp = PermissionMsgResponse(0)
            self.seefinished_flag_temp = True  #识别结束=不需要识别

        return resp

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


if __name__ == "__main__":
    try:
        Master_Car=CarActuator()
        machine=SimpleStateMachine(Master_Car)
        while not rospy.is_shutdown():
            machine.Go()
    except rospy.ROSInterruptException:
        rospy.logerr("程序意外退出")
        pass

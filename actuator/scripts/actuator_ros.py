#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
import actionlib
from actuator.srv import DestinationMsg, DestinationMsgRequest
from actuator.srv import PermissionMsg,PermissionMsgResponse
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point, Pose, Quaternion
import sys
from math import pi
from Announcer import Announcer
import threading

def thread_job():
    rospy.spin()


class CarActuator(object):
    def __init__(self):
        rospy.init_node("car_actuator")
        rospy.on_shutdown(self.actuator_shutdown)

        self.mission_client = rospy.ServiceProxy("mission", DestinationMsg)
        rospy.loginfo("调度器客户端正常启动了")

        self.mission_client.wait_for_service()
        rospy.loginfo("连上调度器服务器了")

        self.permission_server =rospy.Service("permission",PermissionMsg,self.actuator_dealCV_ask)
        rospy.loginfo("命令服务器正常启动")

        # 订阅move_base服务器的消息
        self.move_base_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction
        )
        rospy.loginfo("等待连接move_base服务器")
        self.move_base_client.wait_for_server()
        rospy.loginfo("连上move_base 服务器了")

        self.announcer = Announcer()
        rospy.loginfo("Music on!!!")

        self.status = 1  # 状态机状态
        self.mission_request = DestinationMsgRequest()  # 定义请求
        self.mission_request.car_no = int(sys.argv[1])  # 设定编号

        self.responseToABC = {-1:'E',0: 'A', 1: 'B', 2: 'C'}
        quaternions = list()
        euler_angles = (
            pi / 2,
            pi / 2,
            pi / 2,
             0,
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
        # 创建特殊点列表
        point_ABC = list()
        point_ABC.append(Pose(Point(0.92, 2.53, 0), quaternions[0]))  # A点
        point_ABC.append(Pose(Point(0.92, 3.03, 0), quaternions[1]))  # B点
        point_ABC.append(Pose(Point(0.92, 2.03, 0), quaternions[2]))  # C点

        point_1234 = list()
        point_1234.append(Pose(Point(0,0,0), quaternions[3]))           #特殊点保护
        point_1234.append(Pose(Point(-1.48, 2.13, 0), quaternions[4]))  # 1点
        point_1234.append(Pose(Point(-1.48, 1.63, 0), quaternions[5]))  # 2点
        point_1234.append(Pose(Point(-1.48, 1.13, 0), quaternions[6]))  # 3点
        point_1234.append(Pose(Point(-1.48, 0.63, 0), quaternions[7]))  # 4点

        point_special = list()
        point_special.append(Pose(Point(0, 0, 0), quaternions[8]))  # 起点
        point_special.append(Pose(Point(-0.28, 3.78, 0), quaternions[9]))  # 手写数字识别点

        rospy.loginfo("特殊点创建成功")

        add_thread = threading.Thread(target=thread_job)
        add_thread.start()
        rospy.loginfo("deal CV thread OK")

        while not rospy.is_shutdown():
            # 请求任务相关
            if self.status == 1:
                rospy.loginfo("请求新任务")
                self.actuator_ask_newtarget()
            elif self.status == 2:
                rospy.loginfo("请求成功")
                self.status = 4
            elif self.status == 3:
                rospy.loginfo("请求失败")
                rospy.sleep(5)
                self.actuator_ask_newtarget()

            # 取药相关
            elif self.status == 4:
                rospy.loginfo("前往取药区")
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = point_ABC[self.mission_response.drug_location]
                if self.actuator_move(goal) == True:
                    self.status = 5
                else:
                    rospy.loginfo("取药失败")
                    self.status = 6
            elif self.status == 5:
                newABC = self.responseToABC[self.mission_response.drug_location]
                rospy.loginfo("到达取药区%c", newABC)
                self.actuator_updateABC()  # 上报
                rospy.sleep(2)  # 原地待一会
                self.status = 7

            # 手写数字相关
            elif self.status == 7:
                rospy.loginfo("前往手写数字识别区")
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = point_special[1]
                if self.actuator_move(goal) == True:
                    self.status = 8
                else:
                    rospy.loginfo("手写数字识别区失败")
                    self.status = 9
            elif self.status == 8:
                rospy.loginfo("到达手写数字识别区")
                rospy.sleep(2)  # 原地待一会
                self.status = 10


            # 送药相关
            elif self.status == 10:
                rospy.loginfo("前往送药区")
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = point_1234[self.mission_response.deliver_destination]
                if self.actuator_move(goal) == True:
                    self.status = 11
                else:
                    rospy.loginfo("送药失败")
                    self.status = 12
            elif self.status == 11:
                rospy.loginfo("到达送药区%d",self.mission_response.deliver_destination)
                self.actuator_update1234()
                rospy.sleep(2)  # 原地待一会
                self.status = 13

            # 起点相关
            elif self.status == 13:
                rospy.loginfo("前往起点")
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = point_special[0]
                if self.actuator_move(goal) == True:
                    self.status = 14
                else:
                    rospy.loginfo("前往起点失败")
                    self.status = 15
            elif self.status == 14:
                rospy.loginfo("到达起点")
                self.status = 1

            # 异常相关
            else:
                if self.status != 15:  # 不是前往起点的错误，进入下一状态
                    self.status += 1
                else:
                    self.status = 10  # 回退到上一状态，也就是前往送药区
            
       

    # 程序退出执行
    def actuator_shutdown(self):
        rospy.loginfo("Stop the robot")
        # self.move_base_client.cancel_goal()     #取消当前目标导航点
        rospy.sleep(2)

    # 向服务器请求新任务,到达起点（标准数字区）
    def actuator_ask_newtarget(self):
        self.mission_request.request_type = 1  # 请求包编号为“请求新任务”
        self.mission_response = self.mission_client.call(self.mission_request.car_no, self.mission_request.request_type,0,0)
        rospy.loginfo("车辆代号:%d,请求新任务", self.mission_request.car_no)
        rospy.loginfo(
            "Get:%c,Send:%d",
            self.responseToABC[self.mission_response.drug_location],
            self.mission_response.deliver_destination,
        )
        if (
            self.mission_response.drug_location != -1
            and self.mission_response.deliver_destination != -1
        ):  # 不是负-1代表请求成功
            self.status = 2
        else:
            self.status = 3

    # 向服务器上报已取药
    def actuator_updateABC(self):
        self.mission_request.request_type = 2  # 请求包编号为“完成取药/ABC”
        self.mission_client.call(self.mission_request.car_no, self.mission_request.request_type,0,0)
        self.announcer.arriveDispensingPoint()
 


    # 向服务器上报已送药
    def actuator_update1234(self):
        self.mission_request.request_type = 3  # 请求包编号为“完成送药/1234”
        self.mission_client.call(self.mission_request.car_no, self.mission_request.request_type,0,0)
        self.announcer.arrivePickUpPoint()

    # 向服务器上报已到达手写数字识别区，未增加
    def actuator_arriveHandNum(self):
        self.mission_request.request_type = 4  # 请求包编号为“到达手写数字识别区”
        self.mission_client.call(self.mission_request.car_no, self.mission_request.request_type,0,0)
        

    # 处理来自CV的请求
    def actuator_dealCV_ask(self,req):
        rospy.loginfo("处理识别器请求！")
        if (req.request == 0) : #"想看请求"
            if(self.status == 1 or self.status == 3):    #"已经到达识别区"
                rospy.loginfo("answer OK")
                resp = PermissionMsgResponse(1) #可以看
            else: 
                resp = PermissionMsgResponse(0) #不可以
                rospy.loginfo("answer No!")
        else:                           #"看完了"
            self.status == 10           #离开
            resp = PermissionMsgResponse(0)
            
        return resp
    

    def actuator_move(self, goal):
        # 把目标位置发送给MoveBaseAction的服务器
        self.move_base_client.send_goal(goal)
        # 设定1分钟的时间限制
        finished_within_time = self.move_base_client.wait_for_result(rospy.Duration(45))
        # 如果一分钟之内没有到达，放弃目标
        if not finished_within_time:
            self.move_base_client.cancel_goal()
            if self.status == 4:  # 前往取药时失败
                self.status = 6
            elif self.status == 7:  # 前往手写数字失败
                self.status = 9
            elif self.status == 10:  # 前往送药失败
                self.status = 12
            elif self.status == 13:  # 前往起点失败
                self.status = 15
            # return False
        else:
            state = self.move_base_client.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("成功到达")
                return True
            else:
                rospy.loginfo("没超时但失败")
                return False


if __name__ == "__main__":
    try:
        CarActuator()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("程序意外退出")
        # except KeyboardInterrupt:
        #     rospy.loginfo("程序被键盘打断")
        pass

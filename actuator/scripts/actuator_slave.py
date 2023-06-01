#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
import actionlib
from actuator.srv import DestinationMsg, DestinationMsgRequest
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point, Pose, Quaternion
import sys
from math import pi
from Announcer import Announcer
from std_msgs.msg import Int16
import threading

def thread_job():
    rospy.spin()

class CarActuator(object):
    def __init__(self):
        rospy.init_node("act_slave")
        rospy.on_shutdown(self.actuator_shutdown)

        self.master_status_sub=rospy.Subscriber("/master_status",Int16,self.leavemaster,queue_size=10)
        rospy.loginfo("从机订阅者已上线")
        # 订阅move_base服务器的消息
        self.move_base_client_slave = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction
        )
        rospy.loginfo("等待连接move_base服务器")
        self.move_base_client_slave.wait_for_server()
        rospy.loginfo("连上move_base 服务器了")

        self.mission_client = rospy.ServiceProxy("/mission", DestinationMsg)
        rospy.loginfo("调度器客户端正常启动了")

        self.mission_client.wait_for_service()
        rospy.loginfo("连上调度器服务器了")

        self.announcer = Announcer()
        rospy.loginfo("Music on!!!")

        self.status = 1  # 状态机状态
        self.mission_request = DestinationMsgRequest()  # 定义请求
        self.mission_request.car_no = int(sys.argv[1])  # 设定编号

        self.master_status = 1 #主机状态机状态
        self.updateOnce_flag=0 #0是未上传，1为已上传

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
            -pi / 4,
            pi,
        )
        # 将上面的Euler angles转换成Quaternion的格式
        for angle in euler_angles:
            q_angle = quaternion_from_euler(0, 0, angle, axes="sxyz")
            q = Quaternion(*q_angle)
            quaternions.append(q)
        # 创建特殊点列表
        point_ABC = list()
        point_ABC.append(Pose(Point(0.42, 2.3, 0), quaternions[0]))  # A点
        point_ABC.append(Pose(Point(1.28, 2.73, 0), quaternions[1]))  # B点
        point_ABC.append(Pose(Point(1.28, 1.80, 0), quaternions[2]))  # C点

        point_1234 = list()
        point_1234.append(Pose(Point(0,0,0), quaternions[3]))           #特殊点保护
        point_1234.append(Pose(Point(-1.90, 2.32, 0), quaternions[4]))  # 1点
        point_1234.append(Pose(Point(-1.03, 1.80, 0), quaternions[5]))  # 2点
        point_1234.append(Pose(Point(-1.88, 1.28, 0), quaternions[6]))  # 3点
        point_1234.append(Pose(Point(-1.03, 0.88, 0), quaternions[7]))  # 4点

        point_special = list()
        point_special.append(Pose(Point(-1.75, 0, 0), quaternions[8]))  # 起点
        point_special.append(Pose(Point(-0.4, 3.7, 0), quaternions[9]))  # 手写数字识别点

        rospy.loginfo("特殊点创建成功")

        add_thread = threading.Thread(target=thread_job)
        add_thread.start()
        rospy.loginfo("deal Master thread OK")

        while not rospy.is_shutdown():
            # 请求任务相关
            if self.status == 1:
                rospy.loginfo("请求新任务")
                self.actuator_ask_newtarget()
            elif self.status == 2:
                rospy.loginfo("请求成功")
                if(self.masterstatus>=5):
                    self.status = 4
            elif self.status == 3:
                rospy.loginfo("请求失败")
                rospy.sleep(1)
                self.actuator_ask_newtarget()

            # 取药相关
            elif self.status == 4:
                rospy.loginfo("前往配药区")
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = point_ABC[self.mission_response.drug_location]
                if self.actuator_move(goal) == True:
                    self.status = 5
                else:
                    rospy.loginfo("配药失败")
                    self.status = 6
            elif self.status == 5:
                newABC = self.responseToABC[self.mission_response.drug_location]
                rospy.loginfo("到达配药区%c", newABC)
                if(self.updateOnce_flag ==0):  #未上报
                    self.actuator_updateABC()  #上报
                    self.updateOnce_flag=1     #上报保护
                if(self.masterstatus!=4 or self.masterstatus!=5 or self.masterstatus!=6):
                    self.updateOnce_flag=0     #上报保护
                    rospy.loginfo("主车不在配药区，可以转移")
                    self.status = 7 #转移
                else:
                    rospy.loginfo("主车在配药区，不可转移，请等待")
                    rospy.sleep(2)

            # 手写数字相关,不处理手写数字，不停车
            elif self.status == 7:
                rospy.loginfo("前往手写数字识别区")
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = point_special[1]
                self.move_base_client.send_goal(goal)
                rospy.sleep(4)
                rospy.loginfo("状态转移")
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
                if(self.updateOnce_flag ==0):  #未上报
                    self.actuator_update1234()
                    self.updateOnce_flag=1     #上报保护
                if(self.masterstatus!=10 or self.masterstatus!=11 or self.masterstatus !=12):
                    self.updateOnce_flag=0     #上报保护
                    rospy.loginfo("主车不在送药区，可以转移")
                    self.status = 13 #转移
                else:
                    rospy.loginfo("主车在配药区，不可转移，请等待")
                    rospy.sleep(2)           
            
            # 起点相关
            elif self.status == 13:
                rospy.loginfo("前往起点")
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = point_special[0]
                if(self.masterstatus!= 13 or self.masterstatus!= 14 or self.masterstatus!= 1 or self.masterstatus!= 2 or self.masterstatus!= 3):
                    rospy.loginfo("主车不在起点附近")
                    if self.actuator_move(goal) == True:
                        self.status = 14
                    else:
                        rospy.loginfo("前往起点失败")
                        self.status = 15
                else:
                    rospy.loginfo("主车在起点附近，请等待")
                    rospy.sleep(2)
            elif self.status == 14:
                rospy.loginfo("到达起点")
                self.status = 1

            # 异常相关
            else:
                if self.status != 15:  # 不是前往起点的错误，进入下一状态
                    self.status += 1
                else:
                    self.status = 10  # 回退到上一状态，也就是前往送药区

    def leavemaster(self,msg):
        rospy.loginfo("接收到Master状态为:%d",msg.data)
        self.masterstatus=msg.data
    # 程序退出执行
    def actuator_shutdown(self):
        rospy.loginfo("Stop the robot")
        # self.move_base_client.cancel_goal()     #取消当前目标导航点

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
        self.announcer.arriveDispensingPoint()
        self.mission_request.request_type = 2  # 请求包编号为“完成配药/ABC”
        self.mission_client.call(self.mission_request.car_no, self.mission_request.request_type,0,0)
        
 
    # 向服务器上报已送药
    def actuator_update1234(self):
        self.announcer.arrivePickUpPoint()
        self.mission_request.request_type = 3  # 请求包编号为“完成送药/1234”
        self.mission_client.call(self.mission_request.car_no, self.mission_request.request_type,0,0)


    def actuator_move(self, goal):
        # 把目标位置发送给MoveBaseAction的服务器
        self.move_base_client_slave.send_goal(goal)
        # 设定1分钟的时间限制
        finished_within_time = self.move_base_client_slave.wait_for_result(rospy.Duration(45))
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
            state = self.move_base_client_slave.get_state()
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
        pass

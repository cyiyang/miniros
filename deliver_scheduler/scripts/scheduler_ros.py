# -*- coding: utf-8 -*-
import threading

import rospy
from playsound import playsound
from scheduler import NeedToChangeStatus, RequestType, Scheduler, TargetStatus

from deliver_scheduler.msg import EveryoneStatus
from deliver_scheduler.srv import (
    ChangeTimeResult,
    ChangeTimeResultResponse,
    DestinationMsg,
    DestinationMsgResponse,
    NeedToSeeMsg,
    NeedToSeeMsgResponse,
)

emptyResponse = DestinationMsgResponse()
emptyResponse.drug_location = -1
emptyResponse.deliver_destination = -1


class SchedulerROS(Scheduler):
    def __init__(self, DEBUG=False):
        # 将数字编码类型请求转换为GetNewRequest方法接受的字符类型"A","B","C"
        self.requestDrugTypeToString = {0: "A", 1: "B", 2: "C"}
        self.requestDrugTypeToInt = {"A": 0, "B": 1, "C": 2, None: -1}

        rospy.init_node("scheduler_server")
        super(SchedulerROS, self).__init__(DEBUG)

    def HandleRequests(self, req):
        """处理ROS中Actuator节点请求的方法"""
        response = DestinationMsgResponse()

        rospy.loginfo("[scheduler] %d 号车:", req.car_no)
        if req.request_type == RequestType.GetNewRequest.value:
            rospy.loginfo("[scheduler]收到新的目标板信息!")
            # 将药物类型数字转换为字符"A","B","C"
            stringRequestDrugType = self.requestDrugTypeToString[req.request_drug_type]
            rospy.loginfo(
                "[scheduler] 新的药物需求类型:%s, 配送终点: %d",
                stringRequestDrugType,
                req.request_deliver_destination,
            )
            self.GetNewRequest(stringRequestDrugType, req.request_deliver_destination)
            return emptyResponse

        if req.request_type == RequestType.GetNextTarget.value:
            rospy.loginfo("[scheduler] 收到获取新目标请求!")
            # FUTURE WARNING: Service的requestType和scheduler的requestType含义不同，之后可能会统一
            rospy.loginfo(
                "[scheduler] 当前药物剩余: A:%d, B:%d, C:%d",
                self.GetRemainDrug("A"),
                self.GetRemainDrug("B"),
                self.GetRemainDrug("C"),
            )
            currentQueue = self.GetRequestStatus()
            rospy.loginfo("[scheduler] 当前配送需求:")
            for item in currentQueue:
                rospy.loginfo("[scheduler] %c -> %d", item[0], item[1])

            schedulerResponse, status = self.GetNextTarget(req.car_no)

            # schedulerResponse的requestType字段为请求药物的类型，为字符，需要将其转换为整数
            response.drug_location = self.requestDrugTypeToInt[
                schedulerResponse["requestType"]
            ]
            response.deliver_destination = schedulerResponse["deliverDestination"]

            if status == TargetStatus.DROP_DRUG.value:
                rospy.loginfo("[scheduler] 执行药品清理!")

            if status == TargetStatus.DUPLICATE_REQUEST.value:
                rospy.loginfo("[scheduler] 重复请求,返回上一次任务!")

            # 对于当前无目标的情形，schedulerResponse的两个字段均为None, 在这里用-1表示None
            if response.drug_location is None or response.deliver_destination is None:
                response.drug_location = -1
                response.deliver_destination = -1
                if status == TargetStatus.NO_DRUG_REMAIN.value:
                    rospy.loginfo("[scheduler] 当前没有目标! 因为药品没有库存")
                elif status == TargetStatus.NO_MORE_REQUEST.value:
                    rospy.loginfo("[scheduler] 当前没有目标! 因为没有配送需求")

            rospy.loginfo("[scheduler] 对 actuator 的回复:")
            rospy.loginfo(response)
            return response

        # 对于DrugLoaded和Delivered信息，返回空信息
        if req.request_type == RequestType.DrugLoaded.value:
            rospy.loginfo("Got DrugLoaded!")
            self.DrugLoaded()
            return emptyResponse

        if req.request_type == RequestType.Delivered.value:
            rospy.loginfo("Got Delivered!")
            self.Delivered(req.car_no)
            return emptyResponse

    def RegisterService(self):
        """注册board_reminder客户端服务和mission服务端服务"""
        self.missionServer = rospy.Service(
            "/mission", DestinationMsg, self.HandleRequests
        )
        self.boardReminderClient = rospy.ServiceProxy(
            "board_reminder_server", NeedToSeeMsg
        )
        rospy.loginfo("[reminder] reminder_client正常启动")
        self.boardReminderClient.wait_for_service()
        rospy.loginfo("[reminder] 连接recognizer_server成功")

        self.watcherListenerThread = threading.Thread(target=self.WatcherStatusListener)
        self.watcherListenerThread.setDaemon(True)
        self.watcherListenerThread.start()
        return True

    def WatcherStatusListener(self):
        """订阅话题，并阻塞地等待Watcher到达手写数字点"""

        def CheckTopic(msg):
            if msg.name != "Watcher":
                return
            elif msg.status == "Harbour":
                self.WatcherArrived()

        rospy.loginfo("成功订阅/location话题")

        topicSubscriber = rospy.Subscriber(
            "/location",
            EveryoneStatus,
            CheckTopic,
            queue_size=10,
        )

        rospy.spin()

    def start(self):
        # 启动计时器
        super(SchedulerROS, self).start()

        # 启动监听 watcher 状态线程
        # watcherStatusListenerThread 在 RegisterService 中启动，此处无需处理
        # 执行初始识别任务
        self.BoardRemind()

    def BoardRemind(self):
        rospy.logwarn("[reminder] 去看目标板!")
        self.boardReminderClient.call(True)

    def WatcherArrived(self):
        """当 watcher 到达时，调用该方法，在固定时间后调整药物刷新时间和小哥刷新时间"""
        rospy.loginfo("Watcher到达手写数字点,将在定时时间到达后修改药物刷新时间和小哥刷新时间!")
        self.changeDrugCoolingCountDown.start()
        self.changeNeedToSeeCountDown.start()

    def UpdateNeedToSeeInterval(self, plan):
        rospy.logerr("修改为小哥周期2")
        playsound("/home/EPRobot/Music/DeliverPeriod2.mp3", block=False)
        super(SchedulerROS, self).UpdateNeedToSeeInterval(plan)
        # rospy.logwarn("已修改目标板刷新时间!")

    def UpdateDrugCoolingTime(self, plan):
        rospy.logerr("修改为药品周期3")
        playsound("/home/EPRobot/Music/DrugPeriod3.mp3", block=False)
        super(SchedulerROS, self).UpdateDrugCoolingTime(plan)
        # rospy.logwarn("已修改药物刷新时间!")

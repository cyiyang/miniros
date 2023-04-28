#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys

sys.path.append("/home/ncut/scheduler_ws/devel/lib/python2.7/dist-packages")
from deliver_scheduler.srv import DestinationMsg, DestinationMsgResponse
from scheduler import Scheduler, RequestType
import rospy

DEBUG = 0


def SchedulerServerMain():
    rospy.spin()


def HandleRequests(req):
    response = DestinationMsgResponse()

    if req.request_type == RequestType.GetNewRequest.value:
        rospy.loginfo("Got GetNewRequest!")
        # 将药物类型数字转换为字符"A","B","C"
        stringRequestDrugType = requestDrugTypeToString[req.request_drug_type]
        scheduler.GetNewRequest(stringRequestDrugType, req.request_deliver_destination)
        return emptyResponse

    if req.request_type == RequestType.GetNextTarget.value:
        rospy.loginfo("Got GetNextTarget!")
        # FUTURE WARNING: Service的requestType和scheduler的requestType含义不同，之后可能会统一

        schedulerResponse = scheduler.GetNextTarget()

        # schedulerResponse的requestType字段为请求药物的类型，为字符，需要将其转换为整数
        response.drug_location = requestDrugTypeToInt[schedulerResponse["requestType"]]
        response.deliver_destination = schedulerResponse["deliverDestination"]

        # 对于当前无目标的情形，schedulerResponse的两个字段均为None, 在这里用-1表示None
        if response.drug_location is None:
            response.drug_location = -1
        if response.deliver_destination is None:
            response.drug_location = -1

        rospy.loginfo("Response:")
        rospy.loginfo(response)
        return response

    # 对于DrugLoaded和Delivered信息，返回空信息
    if req.request_type == RequestType.DrugLoaded.value:
        rospy.loginfo("Got DrugLoaded!")
        scheduler.DrugLoaded()
        return emptyResponse

    if req.request_type == RequestType.Delivered.value:
        rospy.loginfo("Got Delivered!")
        scheduler.Delivered()
        return emptyResponse


if __name__ == "__main__":
    scheduler = Scheduler()
    emptyResponse = DestinationMsgResponse()
    emptyResponse.drug_location = -1
    emptyResponse.deliver_destination = -1

    # 将数字编码类型请求转换为GetNewRequest方法接受的字符类型"A","B","C"
    requestDrugTypeToString = {0: "A", 1: "B", 2: "C"}
    requestDrugTypeToInt = {"A": 0, "B": 1, "C": 2}

    rospy.init_node("scheduler_server")
    s = rospy.Service("mission", DestinationMsg, HandleRequests)
    print("Scheduler server ready.")

    SchedulerServerMain()

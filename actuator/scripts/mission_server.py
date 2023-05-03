#! /usr/bin/env python
# -*- coding: utf-8 -*-

# 制作虚假的服务器，进行调试
#模仿scheduler的节点


import rospy
from actuator.srv import DestinationMsg, DestinationMsgRequest, DestinationMsgResponse
import random


def doPrint(request):
    
    response = DestinationMsgResponse()
    rospy.loginfo("车辆编号:%d,请求类型:%s",request.car_no,requestType_dic[request.request_type])
    if (request.request_type ==1):
        response.drug_location = random.randint(0, 2)  # 返回0-2的随机数，代表ABC
        response.deliver_destination = random.randint(0, 3)  # 返回0-3的随机数，代表1-4送药点
        temp = response.deliver_destination+1
        rospy.loginfo("配药位置:%c,送药位置为:%d",
        responseToABC[response.drug_location],
        temp)
    else:
        response.drug_location=-1
        response.deliver_destination=-1

    return response


if __name__ == "__main__":
    requestType_dic={1:"GetNextTarget",2:"DrugLoaded",3:"Delivered"}
    responseToABC = {0: 'A', 1: 'B', 2: 'C'}
    rospy.init_node("scheduler")  # 服务器节点起名为scheduler
    server = rospy.Service("mission", DestinationMsg, doPrint)  # 话题名为mission
    rospy.loginfo("服务器启动")
    rospy.spin()
    pass

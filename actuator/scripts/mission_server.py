#! /usr/bin/env python
# -*- coding: utf-8 -*-

# 制作虚假的服务器，进行调试

import rospy
from actuator.srv import DestinationMsg, DestinationMsgRequest, DestinationMsgResponse
import random


def doPrint(request):
    response = DestinationMsgResponse()
    response.drug_location = random.randint(0, 2)  # 返回0-2的随机数，代表ABC
    response.deliver_destination = random.randint(0, 3)  # 返回0-3的随机数，代表1-4送药点
    rospy.loginfo(
        "接受:%d,%d,发送:%d,%d",
        request.car_no,
        request.request_type,
        response.drug_location,
        response.deliver_destination,
    )
    return response


if __name__ == "__main__":
    rospy.init_node("scheduler")  # 服务器节点起名为scheduler
    server = rospy.Service("mission", DestinationMsg, doPrint)  # 话题名为mission
    rospy.loginfo("服务器启动")
    rospy.spin()
    pass

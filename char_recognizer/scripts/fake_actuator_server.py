#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from char_recognizer.srv import PermissionMsg, PermissionMsgResponse


def doSomething(req):
    response = PermissionMsgResponse()
    if req.request == 0:
        response.permission = 1
        rospy.loginfo("[fake_actuator]收到“想看”, 已回复“能看”")
    else:
        rospy.loginfo("[fake_actuator]收到“看完了”")
    return response

if __name__ == "__main__":
    rospy.init_node("fake_actuator_server.py")
    fake_actuator_server = rospy.Service("permission", PermissionMsg, doSomething)
    rospy.loginfo("[fake_actuator]fake_actuator_server已启动!")
    rospy.spin()
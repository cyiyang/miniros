#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from actuator.srv import PermissionMsg, PermissionMsgRequest, PermissionMsgResponse

if __name__ == "__main__":
    rospy.init_node("CVnode")
    client = rospy.ServiceProxy("permission", PermissionMsg)
    client.wait_for_service()
    req = PermissionMsgRequest()
    rate5 = rospy.Rate(5)
    rate30 = rospy.Rate(30)
    permission_dic = {"Yes": 1, "No": 0}

    while not rospy.is_shutdown():
        resp = client.call(0)
        rospy.loginfo("想看")

        if resp.permission == 0:
            rate5.sleep()
            rospy.loginfo("CV不可看")
        else:
            resp = client.call(1)
            rospy.loginfo("CV看完了")
            rate30.sleep()

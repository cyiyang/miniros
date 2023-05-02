#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time

import rospy
from board_reminder.srv import NeedToSeeMsg, NeedToSeeMsgResponse


class BoardReminderServer:
    def __init__(self):
        rospy.init_node("board_reminder_server")
        self.board_reminder_server = rospy.Service(
            "board_reminder_server", NeedToSeeMsg, self.ReminderHandler
        )
        rospy.loginfo("目标板提示服务器正常启动")
        rospy.spin()

    def ReminderHandler(self, req):
        """在此方法中执行CV相关函数"""
        if req.need_to_see:
            doCV()
        return NeedToSeeMsgResponse(True)


def doCV():
    print("Doing some CV stuff...")
    # 假设处理任务需要一定时间
    time.sleep(5)
    print("CV done!")


if __name__ == "__main__":
    boardReminderServer = BoardReminderServer()

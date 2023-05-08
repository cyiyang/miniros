#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys

sys.path.append("/home/fanch/demo2/src/board_reminder/scripts")

import threading

import rospy
from reminder_basic import Reminder

from board_reminder.srv import NeedToSeeMsg, NeedToSeeMsgResponse


class BoardReminder(Reminder):
    def __init__(self, defaultInterval=3 * 60):
        super(BoardReminder, self).__init__(defaultInterval)
        rospy.init_node("board_reminder")
        self.boardReminderClient = rospy.ServiceProxy(
            "board_reminder_server", NeedToSeeMsg
        )
        rospy.loginfo("[reminder]reminder_client正常启动")
        self.boardReminderClient.wait_for_service()
        rospy.loginfo("[reminder]连接recognizer_server成功")

    def DefaultRemind(self):
        """固定3分钟间隔调用一次，不受其他影响。"""
        # 请求字段为 bool:need_to_see
        rospy.loginfo("[reminder]去看目标板!")
        self.boardReminderClient.call(True)


if __name__ == "__main__":
    boardReminder = BoardReminder()
    rospy.spin()

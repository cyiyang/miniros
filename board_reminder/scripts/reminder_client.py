#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from board_reminder.srv import NeedToSeeMsg, NeedToSeeMsgResponse
from reminder import Reminder


class BoardReminder(Reminder):
    def __init__(self, defaultInterval=3 * 60):
        super(BoardReminder, self).__init__(defaultInterval)
        rospy.init_node("board_reminder")
        self.boardReminderClient = rospy.ServiceProxy(
            "board_reminder_server", NeedToSeeMsg
        )
        rospy.loginfo("目标板提示客户端正常启动")
        self.boardReminderClient.wait_for_service()
        rospy.loginfo("连接目标板提示服务器成功")

    def DefaultRemind(self):
        """固定3分钟间隔调用一次，不受其他影响。"""
        # 请求字段为 bool:need_to_see
        rospy.loginfo("去看目标板!")
        self.boardReminderClient.call(True)


if __name__ == "__main__":
    boardReminder = BoardReminder(10)
    rospy.spin()

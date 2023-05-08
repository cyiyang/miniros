#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys

sys.path.append("/home/fanch/demo2/src/board_reminder/scripts")

import threading
import time

import rospy
from reminder_basic import Reminder

from board_reminder.srv import NeedToSeeMsg, NeedToSeeMsgResponse


class BoardReminder(Reminder):
    def __init__(self, defaultInterval=3 * 60, firstInterval=5):
        rospy.init_node("board_reminder")
        self.boardReminderClient = rospy.ServiceProxy(
            "board_reminder_server", NeedToSeeMsg
        )
        rospy.loginfo("[reminder]reminder_client正常启动")
        self.boardReminderClient.wait_for_service()
        rospy.loginfo("[reminder]连接recognizer_server成功")

        # 在比赛开始时，NeedToSee状态应该为真(检测比赛开始时起点的目标板)，综合考虑其余节点的初始化状态，因此创建一个倒计时5s的一次性定时器来完成该任务
        self.initialTimer = threading.Timer(firstInterval, self.DefaultRemind)

        self.countDownThread = threading.Thread(target=self.NeedToSeeCountDownMain)
        super(BoardReminder, self).__init__(defaultInterval)

    def DefaultRemind(self):
        """固定3分钟间隔调用一次，不受其他影响。"""
        # 请求字段为 bool:need_to_see
        rospy.loginfo("[reminder]去看目标板!")
        self.boardReminderClient.call(True)

    def start(self):
        self.initialTimer.start()
        threading.Thread(target=self.WaitForInitialTimer).start()

    def WaitForInitialTimer(self):
        rospy.loginfo("[reminder] 等待初始识别任务完成...")
        while self.initialTimer.is_alive():
            pass
        rospy.loginfo("初始识别任务完成!")
        super(BoardReminder, self).start()
        self.countDownThread.start()

    def NeedToSeeCountDownMain(self):
        self.nextNeedToSeeCounter = self.defaultInterval
        while self.running:
            time.sleep(5)
            self.nextNeedToSeeCounter -= 5
            rospy.loginfo("[reminder] 下一次NeedToSee还有 %ds", self.nextNeedToSeeCounter)
            if self.nextNeedToSeeCounter <= 0:
                self.nextNeedToSeeCounter = self.defaultInterval


if __name__ == "__main__":
    boardReminder = BoardReminder()
    rospy.spin()

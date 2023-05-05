#!/usr/bin/env python
# -*- coding: utf-8 -*-


import threading
import time
import warnings

DEBUGING = True

class Reminder(object):
    def __init__(self, defaultInterval=3 * 60):
        self.defaultInterval = defaultInterval
        # Python 的 Timer 是一次性的，因此周期性任务需要在上一个 Timer 结束时释放出新的 Timer,
        # 考虑到 ROS 等待服务返回是阻塞的，因此每次均新建一个 Timer, 而不是在旧的 Timer 的回调函数中重新开始 Timer.
        # timerUpdater 负责定时释放出新的 Timer
        self.timerUpdater = threading.Timer(
            self.defaultInterval, self.CreateNewDefaultTimer
        )
        self.running = True
        self.timerUpdater.start()
        self.defaultTimer = threading.Timer(self.defaultInterval, self.DefaultRemind)
        self.defaultTimer.start()
        self.timerUpdateThread = threading.Thread(target=self.ReminderMain)
        self.timerUpdateThread.start()

    def DefaultRemind(self):
        """每三分钟触发一次该函数。"""
        if DEBUGING:
            print("Triggered Default Remind, but do nothing!")
        else:
            raise NotImplementedError("请在子类中实现该方法")

    def ReminderMain(self):
        while self.running:
            if self.timerUpdater.is_alive():
                pass
            else:
                self.timerUpdater = threading.Timer(
                    self.defaultInterval, self.CreateNewDefaultTimer
                )
                self.timerUpdater.start()
                print("Release new timerUpdater!")
            # if self.fullBoard:

    def CreateNewDefaultTimer(self):
        timer = threading.Timer(self.defaultInterval, self.DefaultRemind)
        timer.start()
        print("Release new DefaultTimer!")
        return timer

    def FullBoardUpdateReminder(self):
        """当有目标板满和小哥排队引起的目标板不定时更新时，发出提醒。"""
        if DEBUGING:
            print("Triggered FullBoard Update Reminder, but do nothing!")
        else:
            raise NotImplementedError("请在子类中实现该方法")

    def Terminate(self):
        self.running = False


if __name__ == "__main__":
    reminder = Reminder(5)
    time.sleep(15)
    reminder.Terminate()

#!/usr/bin/env python
# -*- coding: utf-8 -*-


import threading
import time
import warnings

DEBUGING = True


class Reminder(object):
    """每隔Interval时间非阻塞调用DefaultRemind方法的定时器对象,第一个定时器的等待时间为firstInterval"""

    def __init__(self, defaultInterval=3 * 60):
        self.defaultInterval = defaultInterval
        # Python 的 Timer 是一次性的，因此周期性任务需要在上一个 Timer 结束时释放出新的 Timer,
        # 考虑到 ROS 等待服务返回是阻塞的，因此每次均新建一个 Timer, 而不是在旧的 Timer 的回调函数中重新开始 Timer.
        # timerUpdater 负责定时释放出新的 Timer

        # 维护三分钟定时器的定时器
        self.defaultTimerUpdater = threading.Timer(
            self.defaultInterval, self.CreateNewDefaultTimer
        )
        self.running = True
        self.timerUpdateThread = threading.Thread(target=self.ReminderMain)

    def start(self):
        # 启动之前定义的定时器和线程
        self.CreateNewDefaultTimer()
        self.defaultTimerUpdater.start()
        self.timerUpdateThread.start()

    def DefaultRemind(self):
        """每三分钟触发一次该函数。"""
        if DEBUGING:
            print("Triggered Default Remind, but do nothing!")
        else:
            raise NotImplementedError("请在子类中实现该方法")

    def ReminderMain(self):
        """检查各个定时器产生器的运行状态，并更新失效的定时器"""
        while self.running:
            # 检查配送目标板提示定时器的运行状态，如定时已结束，则新建配送目标板提示定时器
            if self.defaultTimerUpdater.is_alive():
                pass
            else:
                self.defaultTimerUpdater = threading.Timer(
                    self.defaultInterval, self.CreateNewDefaultTimer
                )
                self.defaultTimerUpdater.start()
                print("Release new timerUpdater!")
            # if self.fullBoard:

    def CreateNewDefaultTimer(self):
        """创建新的三分钟定时器"""
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


class AutoReloadTimer(object):
    def __init__(self, interval, function, argsToFunction=None, autoReload=True):
        self.interval = interval
        self.function = function
        self.argsToFunction = argsToFunction
        self.autoReload = autoReload
        self.running = True
        self.__argsLock = threading.Lock()
        self.__timer = threading.Timer(interval=self.interval, function=self.TimesUp)
        self.__timer.start()

    def TimesUp(self):
        """计时结束时，新建线程执行function，并更新Timer"""
        if self.running:
            with self.__argsLock:
                # 将参数传递给function，新建线程处理function
                functionHandlerThread = threading.Thread(
                    target=self.function, args=self.argsToFunction
                )
                functionHandlerThread.start()
            if self.autoReload:
                # 自动重载定时器
                self.__timer = threading.Timer(
                    interval=self.interval, function=self.TimesUp
                )
                self.__timer.start()

    def UpdateArgsToFunction(self, newArgs):
        with self.__argsLock:
            self.argsToFunction = newArgs
        if self.__timer.is_alive():
            return True
        else:
            # 如果定时器已经执行完成，则更新参数无效，返回失败
            return False


if __name__ == "__main__":
    reminder = Reminder(5)
    time.sleep(15)
    reminder.Terminate()

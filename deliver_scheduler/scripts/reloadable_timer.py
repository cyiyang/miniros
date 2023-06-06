#!/usr/bin/env python
# -*- coding: utf-8 -*-

import threading
import time


class ReloadableTimer(object):
    def __init__(self, interval, autoReload, function, args=[], kwargs={}):
        self.interval = interval
        self.reloadInterval = interval
        self.function = function
        self.args = args
        self.kwargs = kwargs
        self.thread = None
        self.running = False
        self.startTime = None
        self.autoReload = autoReload

    def start(self):
        if self.running:
            return
        self.running = True
        self.startTime = time.time()
        self.thread = threading.Timer(self.interval, self._run)
        # setDaemon(True) 后，当主线程退出时，其他定时器线程也将退出
        self.thread.setDaemon(True)
        self.thread.start()

    def _run(self):
        self.running = False
        self.startTime = None
        threading.Thread(target=self.function(*self.args, **self.kwargs)).start()
        if self.autoReload:
            # 自动重载会以 self.reloadInterval 指定的间隔进行
            self.restart(self.reloadInterval)

    def stop(self):
        self.thread.cancel()
        self.running = False

    def restart(self, interval=None):
        """使得定时器的剩余时间为 interval，但不影响定时器的重载值"""
        if interval is not None:
            self.interval = interval
        if self.thread is not None:
            self.stop()
        self.start()

    def restartWithInterval(self, interval):
        """设置定时器的计时周期，并立刻以该周期重新开始计时"""
        self.interval = interval
        self.reloadInterval = interval
        if self.thread is not None:
            self.stop()
        self.start()

    def isAlive(self):
        return self.thread and self.running

    def getElapsedTime(self):
        """若没有执行过 setRemainTime, 该方法会返回start后的时间，否则会返回上一次 setRemainTime 后经过的时间"""
        if not self.startTime:
            return None
        return time.time() - self.startTime

    def getRemainTime(self):
        """返回本次定时剩余的时间, 如果定时器没有启动，则会返回 None"""
        if not self.startTime:
            return None
        return max(0, self.interval - self.getElapsedTime())

    def setRemainTime(self, remainTime):
        """临时修改定时器的剩余时间，本轮定时器超时后，将按原有的周期继续运行"""
        self.restart(remainTime)

    def setNewInterval(self, newInterval):
        """修改下一周期开始的定时周期，不会影响当前周期的剩余时间"""
        self.reloadInterval = newInterval

    def getReloadInterval(self):
        return self.reloadInterval

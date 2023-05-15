# -*- coding: utf-8 -*-
import threading
import time
from enum import Enum, unique

from statemachine import State, StateMachine
from statemachine.exceptions import TransitionNotAllowed


class Scheduler:
    """
    调度器对象
    """

    def __init__(self, DEBUG=False):
        """
        queue中的元素为一字典,具有字段:
        priority: 搬运的优先级，为关于targetType和elapsedTime的函数
        requestType: 快递小哥的需求类型
        deliverDestination: 需求需要被送往的位置
        elapsedTime: 快递小哥的需求被检测到以来经过的时间
        """
        self.queue = []
        self.classWeights = {"A": 20, "B": 15, "C": 10}
        self.CAR_CNT = 2
        self.nextTarget = [None] * self.CAR_CNT
        self.queueLock = threading.Lock()  # May be implemented
        self.DEBUG = DEBUG

        # 起始时三种药各有一瓶
        self.drugRemain = {"A": 1, "B": 1, "C": 1}
        self.drugRemainLock = threading.Lock()

        # self.drugSupplementInterval = {"A": 120, "B": 60, "C": 40}
        if DEBUG:
            self.drugSupplementInterval = {"A": 12, "B": 6, "C": 4}
        else:
            self.drugSupplementInterval = {"A": 120, "B": 60, "C": 40}

        self.__noTarget = {
            "requestType": None,
            "deliverDestination": None,
        }

        # self.__threads = []
        # self.__threads.append(
        #     threading.Thread(target=self.__DrugTracerMain, args=("A"))
        # )
        # self.__threads.append(
        #     threading.Thread(target=self.__DrugTracerMain, args=("B"))
        # )
        # self.__threads.append(
        #     threading.Thread(target=self.__DrugTracerMain, args=("C"))
        # )

        # self.running = True
        # for thread in self.__threads:
        #     thread.start()

        self.coolingTimeStateMachine = CoolingTime()

        self.timers = [
            ReloadableTimer(interval, True, self.__UpdateRemainDrug, [drugType, 1])
            for drugType, interval in self.drugSupplementInterval.items()
        ]
        for timer in self.timers:
            timer.start()

    def GetNextTarget(self, car_id=0):
        """获取下个目标

        Returns:
        一个包含字段requestType和deliverDestination的字典。
            例如:
            {"requestType": "A", "deliverDestination": 1}
            {"requestType": None, "deliverDestination": None}
        """
        with self.queueLock:
            if self.nextTarget[car_id] is None:
                if self.queue:
                    # 小哥有取药需求时，更新优先级
                    self.__UpdatePriority()
                    for index, target in enumerate(self.queue):
                        # TODO: 实现优先级高的药物不可用时切换至下一类药物
                        if self.GetRemainDrug(target["requestType"]) >= 1:
                            self.nextTarget[car_id] = self.queue.pop(index)
                            # 为了避免小车对同一送药需求、不同配送目的地导致的“抢药”问题，将药物的更新放在这里处理
                            self.__UpdateRemainDrug(
                                self.nextTarget[car_id]["requestType"], -1
                            )
                            return self.nextTarget[car_id], TargetStatus.SUCCESS.value
                    # 循环正常结束，表明需求的药物现在都没有，应返回无目标
                    return self.__noTarget, TargetStatus.NO_DRUG_REMAIN.value
                    # self.nextTarget = self.queue.pop()
                else:
                    # 小哥没有取药需求
                    return self.__noTarget, TargetStatus.NO_MORE_REQUEST.value
            else:
                return self.nextTarget[car_id], TargetStatus.SUCCESS.value

    def Delivered(self, car_id=0):
        """
        送达药物时调用该函数
        """
        if self.nextTarget[car_id] is None:
            # 多次调用的情况直接返回
            return

        self.nextTarget[car_id] = None

    def GetNewRequest(self, requestType, deliverDestination):
        """
        当获得新的快递小哥需求时，调用此函数。
        """
        with self.queueLock:
            requestDetail = {
                "priority": self.classWeights[requestType],
                "requestType": requestType,
                "deliverDestination": deliverDestination,
                "elapsedTime": 0,
            }
            for item in self.queue:
                if (
                    item["requestType"] == requestDetail["requestType"]
                    and item["deliverDestination"]
                    == requestDetail["deliverDestination"]
                ):
                    # 如果两个字段与已有内容完全相同，表明是上一轮遗留的任务
                    return
            self.queue.append(requestDetail)

    def __UpdatePriority(self, timeNow=time.time()):
        """
        周期性运行，更新队伍中优先级
        """
        # TODO: 实现根据时间调整优先级
        # 此处不能加锁，否则在 GetNextTarget 中调用该函数时会导致死锁；因此将该方法变成私有方法
        self.queue.sort(key=lambda x: x["priority"], reverse=True)

    def DrugLoaded(self, car_id=0):
        """当小车拾取药物，调用该函数。该函数现在不会进行任何操作"""
        # self.__UpdateRemainDrug(self.nextTarget[car_id]["requestType"], -1)
        return

    def __DrugTracerMain(self, drugType):
        """周期性唤醒，使得某种药物的存量+1"""
        while self.running:
            time.sleep(self.drugSupplementInterval[drugType])
            self.__UpdateRemainDrug(drugType, 1)

    def __UpdateRemainDrug(self, drugType, addend):
        """带有锁机制的药物更新。更新后drugType类型药物的数量为原数量+addend."""
        with self.drugRemainLock:
            self.drugRemain[drugType] += addend

    def GetRemainDrug(self, drugType):
        with self.drugRemainLock:
            return self.drugRemain[drugType]

    def GetRequestStatus(self):
        """返回当前优先级队列信息，包括需求类型和配送目的地"""
        res = []
        with self.queueLock:
            res = [
                (item["requestType"], item["deliverDestination"]) for item in self.queue
            ]
            # for item in self.queue:
            #     res.append((item["requestType"], item["deliverDestination"]))
        return res

    def Terminate(self):
        self.running = False

    def GetNeedToChangeStatus(self):
        """获取当前是否需要修改配送周期"""
        with self.queueLock:
            haveRequestForA = False
            for request in self.queue:
                if request["requestType"] == "A":
                    haveRequestForA = True
            if self.GetRemainDrug("A") <= 0:
                noRemainForA = True
            else:
                noRemainForA = False
            if self.GetRemainDrug("B") >= 3:
                overflowForB = True
            if self.GetRemainDrug("C") >= 3:
                overflowForC = True
        if haveRequestForA and noRemainForA:
            return NeedToChangeStatus.SPEED_UP.value
        elif not noRemainForA and (overflowForB or overflowForC):
            # B或C发生堆积而且A有剩余时，应减缓药物刷新
            return NeedToChangeStatus.SLOW_DOWN.value
        else:
            return NeedToChangeStatus.DONT_CHANGE.value

    def UpdateDrugCoolingTime(self, needToChangeStatus):
        """在正确识别手写数字后，更新药物的刷新时间
        @needToChangeStatus: 修改刷新时间的方向，可以为 DONT_CHANGE, SPEED_UP, SLOW_DOWN (.value)
        @return: True, 当转换成功; False, 当转换失败
        """
        if needToChangeStatus == NeedToChangeStatus.DONT_CHANGE.value:
            return True
        if needToChangeStatus == NeedToChangeStatus.SPEED_UP.value:
            try:
                self.coolingTimeStateMachine.send("SPEED_UP")
            except TransitionNotAllowed:
                if not self.DEBUG:
                    print("不允许的状态转换!")
                else:
                    raise ValueError("不允许的状态转换!")
                return False
            for timer in self.timers:
                timer.setRemainTime(timer.getRemainTime() / 2.0)
                timer.setNewInterval(timer.getReloadInterval() / 2.0)
        elif needToChangeStatus == NeedToChangeStatus.SLOW_DOWN.value:
            try:
                self.coolingTimeStateMachine.send("SLOW_DOWN")
            except TransitionNotAllowed:
                print("不允许的状态转换!")
                return False
            for timer in self.timers:
                timer.setRemainTime(timer.getRemainTime() * 2)
                timer.setNewInterval(timer.getReloadInterval() * 2)

    def ForgiveCurrentTask(self, car_no):
        return


@unique
class RequestType(Enum):
    GetNewRequest = 0
    GetNextTarget = 1
    DrugLoaded = 2
    Delivered = 3


TargetStatus = Enum("TargetStatus", ("SUCCESS", "NO_DRUG_REMAIN", "NO_MORE_REQUEST"))


class ReloadableTimer:
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


NeedToChangeStatus = Enum(
    "NeedToChangeStatus", ("SPEED_UP", "DONT_CHANGE", "SLOW_DOWN")
)


class CoolingTime(StateMachine):
    # 定义状态
    speedUpState = State("speedUpState")
    slowDownState = State("slowDownState")
    normalState = State("normalState", initial=True)

    # 定义状态转换
    SPEED_UP = slowDownState.to(normalState) | normalState.to(speedUpState)
    SLOW_DOWN = speedUpState.to(normalState) | normalState.to(slowDownState)

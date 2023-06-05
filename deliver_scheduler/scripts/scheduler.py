# -*- coding: utf-8 -*-
import threading
import time
from enum import Enum, unique

from reloadable_timer import ReloadableTimer
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
        self.queueLock = threading.RLock()
        self.DEBUG = DEBUG

        # 三种方案下的药物刷新时间
        self.DRUG_PERIOD = {
            CoolingTimePlan.PERIOD_1: {"A": 120, "B": 60, "C": 40},
            CoolingTimePlan.PERIOD_2: {"A": 60, "B": 50, "C": 20},
            CoolingTimePlan.PERIOD_3: {"A": 40, "B": 30, "C": 15},
        }

        # 起始时三种药各有一瓶
        self.drugRemain = {"A": 1, "B": 1, "C": 1}
        self.drugRemainLock = threading.RLock()

        # self.drugSupplementInterval = {"A": 120, "B": 60, "C": 40}
        if DEBUG:
            self.drugSupplementInterval = {"A": 12, "B": 6, "C": 4}
        else:
            # 国赛版本
            self.drugSupplementInterval = {"A": 180, "B": 120, "C": 60}

        self.__noTarget = {
            "requestType": None,
            "deliverDestination": None,
        }

        self.__dropDrugTarget = {
            "requestType": "C",
            "deliverDestination": 5,
        }

        self.coolingTimeStateMachine = CoolingTime()

        # 定义记录药物补充的定时器
        self.drugSupplementTimers = {
            drugType: ReloadableTimer(
                interval, True, self.__UpdateRemainDrug, [drugType, 1]
            )
            for drugType, interval in self.drugSupplementInterval.items()
        }
        for drugType, timer in self.drugSupplementTimers.items():
            timer.start()

    def GetNextTarget(self, car_id=0):
        """获取下个目标

        Returns:
        一个包含字段requestType和deliverDestination的字典。
            例如:
            {"requestType": "A", "deliverDestination": 1}
            {"requestType": None, "deliverDestination": None}
        """
        targetStatus = TargetStatus.SUCCESS.value

        if self.nextTarget[car_id] is None:
            with self.queueLock:
                if self.queue:
                    # 小哥有取药需求时，更新优先级
                    self.__UpdatePriority()
                    for index, target in enumerate(self.queue):
                        # 当小哥要求的药物有剩余时，可分配任务
                        if self.GetRemainDrug(target["requestType"]) >= 1:
                            # 将目标赋予请求任务的车辆
                            self.nextTarget[car_id] = self.queue.pop(index)

                            # 为了避免小车对同一送药需求、不同配送目的地导致的“抢药”问题，将药物的更新放在这里处理
                            self.__UpdateRemainDrug(
                                self.nextTarget[car_id]["requestType"], -1
                            )

                            return self.nextTarget[car_id], TargetStatus.SUCCESS.value

                    # 循环正常结束，表明需求的药物现在都没有
                    targetStatus = TargetStatus.NO_DRUG_REMAIN.value
                else:
                    # 队列为空，表明小哥没有取药需求
                    targetStatus = TargetStatus.NO_MORE_REQUEST.value

            # 没有正常目标时，处理多余的药物
            with self.drugRemainLock:
                # 获取存量最多的药物的类型和存量
                mostStockedDrugType, mostStockedDrugAmount = max(
                    self.drugRemain.items(), key=lambda x: x[1]
                )
                if mostStockedDrugAmount < 3:
                    # 如果存量最多的药物不足3，不需要清理
                    pass
                else:
                    # 将存量药物最多的存量-1
                    self.__UpdateRemainDrug(mostStockedDrugType, -1)
                    self.nextTarget[car_id] = self.__dropDrugTarget
                    self.nextTarget[car_id]["requestType"] = mostStockedDrugType
                    return self.nextTarget[car_id], TargetStatus.DROP_DRUG.value

            # 当也没有药物需要清理时，返回无目标
            return self.__noTarget, targetStatus

        else:
            # 小车多次请求目标，返回上一次的目标
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
        # 可重入锁允许在单一线程中多次获取锁，而不引起死锁
        with self.queueLock:
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
            overflowForB = False
            overflowForC = False
            for request in self.queue:
                if request["requestType"] == "A":
                    haveRequestForA = True
                    break
            if self.GetRemainDrug("A") <= 0:
                noRemainForA = True
            else:
                noRemainForA = False
            if self.GetRemainDrug("B") >= 3:
                overflowForB = True
            if self.GetRemainDrug("C") >= 3:
                overflowForC = True
        if (
            haveRequestForA
            and noRemainForA
            and self.coolingTimeStateMachine.current_state.value != "speedUpState"
        ):
            return NeedToChangeStatus.SPEED_UP.value
        elif (
            (not noRemainForA)
            and (overflowForB or overflowForC)
            and self.coolingTimeStateMachine.current_state.value != "slowDownState"
        ):
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
                    # 避免程序直接退出
                    raise ValueError("不允许的状态转换!")
                return False
            for timer in self.drugSupplementTimers:
                # 在当前的基础上以 50% 的比率提高或降低三个配药窗口的配送周期，即变为原时间的 1 - 0.5 = 0.5 倍
                timer.setRemainTime(timer.getRemainTime() / 2.0)
                timer.setNewInterval(timer.getReloadInterval() / 2.0)
        elif needToChangeStatus == NeedToChangeStatus.SLOW_DOWN.value:
            try:
                self.coolingTimeStateMachine.send("SLOW_DOWN")
            except TransitionNotAllowed:
                print("不允许的状态转换!")
                return False
            for timer in self.drugSupplementTimers:
                # 在当前的基础上以 50% 的比率提高或降低三个配药窗口的配送周期，即提升至原时间的 1 + 0.5 = 1.5 倍
                timer.setRemainTime(timer.getRemainTime() * (1 + 0.5))
                timer.setNewInterval(timer.getReloadInterval() * (1 + 0.5))

    def ForgiveCurrentTask(self, car_no):
        return

    def SetDrugCoolingTime(self, plan):
        """设置药物刷新时间为周期 1, 2 或 3"""
        for drugType, newInterval in self.DRUG_PERIOD[plan].items():
            self.drugSupplementTimers[drugType].restartWithInterval(newInterval)


@unique
class RequestType(Enum):
    GetNewRequest = 0
    GetNextTarget = 1
    DrugLoaded = 2
    Delivered = 3


TargetStatus = Enum(
    "TargetStatus", ("SUCCESS", "NO_DRUG_REMAIN", "NO_MORE_REQUEST", "DROP_DRUG")
)


NeedToChangeStatus = Enum(
    "NeedToChangeStatus", ("SPEED_UP", "DONT_CHANGE", "SLOW_DOWN")
)

CoolingTimePlan = Enum("CoolingTimePlan", ("PERIOD_1", "PERIOD_2", "PERIOD_3"))

NeedToSeePlan = Enum("NeedToSeePlan", ("PERIOD_1", "PERIOD_2", "PERIOD_3"))


class CoolingTime(StateMachine):
    # 定义状态
    speedUpState = State("speedUpState")
    slowDownState = State("slowDownState")
    normalState = State("normalState", initial=True)

    # 定义状态转换
    SPEED_UP = slowDownState.to(normalState) | normalState.to(speedUpState)
    SLOW_DOWN = speedUpState.to(normalState) | normalState.to(slowDownState)

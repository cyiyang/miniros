# -*- coding: utf-8 -*-
import threading
import time
from enum import Enum, unique


class Scheduler:
    """
    调度器对象
    """

    def __init__(self):
        """
        queue中的元素为一字典,具有字段:
        priority: 搬运的优先级，为关于targetType和elapsedTime的函数
        requestType: 快递小哥的需求类型
        deliverDestination: 需求需要被送往的位置
        elapsedTime: 快递小哥的需求被检测到以来经过的时间
        """
        self.queue = []
        self.classWeights = {"A": 20, "B": 15, "C": 10}
        self.nextTarget = None
        self.queueLock = threading.Lock()  # May be implemented

        # 起始时三种药各有一瓶
        self.drugRemain = {"A": 1, "B": 1, "C": 1}
        self.drugRemainLock = threading.Lock()

        # self.drugSupplementInterval = {"A": 120, "B": 60, "C": 40}
        self.drugSupplementInterval = {"A": 12, "B": 6, "C": 4}

        self.__noTarget = {
            "requestType": None,
            "deliverDestination": None,
        }

        self.__threads = []
        self.__threads.append(
            threading.Thread(target=self.__DrugTracerMain, args=("A"))
        )
        self.__threads.append(
            threading.Thread(target=self.__DrugTracerMain, args=("B"))
        )
        self.__threads.append(
            threading.Thread(target=self.__DrugTracerMain, args=("C"))
        )

        self.running = True
        for thread in self.__threads:
            thread.start()
        # 等待线程结束

    def GetNextTarget(self):
        """获取下个目标

        Returns:
        一个包含字段requestType和deliverDestination的字典。
            例如:
            {"requestType": "A", "deliverDestination": 1}
            {"requestType": None, "deliverDestination": None}
        """

        if self.nextTarget is None:
            if self.queue:
                # 小哥有取药需求时，更新优先级
                self.UpdatePriority()
                for index, target in enumerate(self.queue):
                    # TODO: 实现优先级高的药物不可用时切换至下一类药物
                    if (
                        self.GetRemainDrug(target["requestType"], self.drugRemainLock)
                        >= 1
                    ):
                        self.nextTarget = self.queue.pop(index)
                        return self.nextTarget, TargetStatus.SUCCESS.value
                # 循环正常结束，表明需求的药物现在都没有，应返回无目标
                return self.__noTarget, TargetStatus.NO_DRUG_REMAIN.value
                # self.nextTarget = self.queue.pop()
            else:
                # 小哥没有取药需求
                return self.__noTarget, TargetStatus.NO_MORE_REQUEST.value

        # return {
        #     "requestType": self.nextTarget["requestType"],
        #     "deliverDestination": self.nextTarget["deliverDestination"],
        # }

    def Delivered(self):
        """
        送达药物时调用该函数
        """
        if self.nextTarget is None:
            # 多次调用的情况直接返回
            return

        self.nextTarget = None

    def GetNewRequest(self, requestType, deliverDestination):
        """
        当获得新的快递小哥需求时，调用此函数。
        """
        requestDetail = {
            "priority": self.classWeights[requestType],
            "requestType": requestType,
            "deliverDestination": deliverDestination,
            "elapsedTime": 0,
        }
        self.queue.append(requestDetail)

    def UpdateRequestPeriod(self):
        """
        获取是否需要调整取药周期
        """
        # To be implemented
        return False

    def UpdatePriority(self, timeNow=time.time()):
        """
        周期性运行，更新队伍中优先级
        """
        # TODO: 实现根据时间调整优先级
        self.queue.sort(key=lambda x: x["priority"], reverse=True)

    def DrugLoaded(self):
        """当小车拾取药物，调用该函数。该函数会将nextTarget对应的剩余药物量-1"""
        self.__UpdateRemainDrugWithLock(
            self.nextTarget["requestType"], -1, self.drugRemainLock
        )

    def __DrugTracerMain(self, drugType):
        """周期性唤醒，使得某种药物的存量+1"""
        while self.running:
            time.sleep(self.drugSupplementInterval[drugType])
            self.__UpdateRemainDrugWithLock(drugType, 1, self.drugRemainLock)
            # self.__GetRemainDrugWithLock(drugType,self.drugRemainLock)
            # print(
            #     f"{drugType} updated! Remaining {drugType} for {self.__GetRemainDrugWithLock(drugType,self.drugRemainLock)}"
            # )

    def __UpdateRemainDrugWithLock(self, drugType, addend, lock):
        """带有锁机制的药物更新。更新后drugType类型药物的数量为原数量+addend."""
        with self.drugRemainLock:
            self.drugRemain[drugType] += addend

    def GetRemainDrug(self, drugType):
        with self.drugRemainLock:
            return self.drugRemain[drugType]

    def Terminate(self):
        self.running = False


@unique
class RequestType(Enum):
    GetNewRequest = 0
    GetNextTarget = 1
    DrugLoaded = 2
    Delivered = 3


TargetStatus = Enum("TargetStatus", ("SUCCESS", "NO_DRUG_REMAIN", "NO_MORE_REQUEST"))

import time


class Scheduler:
    def __init__(self):
        """
        调度器对象
        """

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
        self.queueLock = None  # May be implemented

    def GetNextTarget(self):
        """
        获取下个目标
        """
        if self.nextTarget is None:
            if self.queue:
                self.UpdatePriority()
                self.nextTarget = self.queue.pop()
            else:
                return None

        return {
            "requestType": self.nextTarget["requestType"],
            "deliverDestination": self.nextTarget["deliverDestination"],
        }

    def Delivered(self):
        """
        送达药物时调用该函数
        """
        self.nextTarget = None

    def GetNewRequest(self, requestType, deliverDestination):
        """
        当获得新的快递小哥需求时，调用此函数
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
        self.queue.sort(key=lambda x: x["priority"])

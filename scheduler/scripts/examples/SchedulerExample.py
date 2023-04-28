"""
流程: GetNextTarget -> DrugLoaded -> Delivered
"""

import sys

sys.path.append("..")

import os

os.chdir(os.path.dirname(__file__))
import time

from scheduler import Scheduler

if __name__ == "__main__":
    scheduler = Scheduler()

    # 获取将A药品送到区域1的需求
    scheduler.GetNewRequest("A", 1)

    target = scheduler.GetNextTarget()
    target1 = scheduler.GetNextTarget()
    # 在Delivered前多次调用GetNextTarget会返回相同的目标，也就是说搬完一个药品并调用Delivered才会指派下一个搬运的药品
    assert target == target1
    print(target)

    # Do something if you like
    time.sleep(0.1)

    # 多次调用Delivered没有后果
    scheduler.Delivered()
    scheduler.Delivered()

    scheduler.GetNewRequest("C", 2)
    scheduler.GetNewRequest("B", 3)

    target = scheduler.GetNextTarget()
    print(target)

    scheduler.Delivered()
    target = scheduler.GetNextTarget()
    print(target)
    time.sleep(0.1)
    scheduler.Delivered()

    # 当没有需求时，返回None
    target = scheduler.GetNextTarget()
    print(target)

    while True:
        pass
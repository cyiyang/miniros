"""回归测试，修改后的Scheduler应通过该测试
"""

import sys

sys.path.append("..")

import os

os.chdir(os.path.dirname(__file__))
import time

from scheduler import Scheduler

if __name__ == "__main__":
    scheduler = Scheduler()
    scheduler.GetNewRequest("A", 1)
    nextTarget = scheduler.GetNextTarget()
    scheduler.DrugLoaded()
    time.sleep(5)
    scheduler.Delivered()
    scheduler.GetNewRequest("B", 2)
    scheduler.GetNewRequest("C", 4)
    scheduler.GetNewRequest("A", 1)
    nextTarget = scheduler.GetNextTarget()
    # 药物A无剩余，此时应该取药物B
    assert nextTarget["requestType"] == "B" and nextTarget["deliverDestination"] == 2

    time.sleep(8)
    # 等待药物A补充，同时送达当前药物
    scheduler.Delivered()
    scheduler.GetNewRequest("B", 3)
    nextTarget = scheduler.GetNextTarget()
    assert nextTarget["requestType"] == "A" and nextTarget["deliverDestination"] == 1
    scheduler.DrugLoaded()
    scheduler.Terminate()
    time.sleep(1)
    print("通过测试!")
    exit(0)

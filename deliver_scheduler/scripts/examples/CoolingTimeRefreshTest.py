# -*- coding: utf-8 -*-
"""回归测试，修改后的Scheduler应通过该测试
"""

import sys

sys.path.append("..")

import os

os.chdir(os.path.dirname(__file__))
import time

from scheduler import Scheduler, NeedToChangeStatus

if __name__ == "__main__":
    scheduler = Scheduler(DEBUG=True)
    scheduler.GetNewRequest("A", 1)
    nextTarget, _ = scheduler.GetNextTarget()
    assert nextTarget["requestType"] == "A" and nextTarget["deliverDestination"] == 1
    time.sleep(4)
    scheduler.DrugLoaded()
    assert scheduler.GetRemainDrug("A") == 0
    time.sleep(2)
    assert scheduler.GetNeedToChangeStatus() == NeedToChangeStatus.DONT_CHANGE.value
    time.sleep(4)
    scheduler.Delivered()
    assert scheduler.GetRemainDrug("C") == 3
    time.sleep(3)
    assert scheduler.GetNeedToChangeStatus() == NeedToChangeStatus.SLOW_DOWN.value
    scheduler.GetNewRequest("A", 2)
    scheduler.GetNewRequest("B", 3)
    scheduler.GetNewRequest("A", 4)
    nextTarget, _ = scheduler.GetNextTarget()
    assert nextTarget["requestType"] == "A" and nextTarget["deliverDestination"] == 2
    time.sleep(3)
    scheduler.DrugLoaded()
    assert scheduler.GetRemainDrug("A") == 0
    time.sleep(1)
    assert scheduler.GetNeedToChangeStatus() == NeedToChangeStatus.SPEED_UP.value
    scheduler.UpdateDrugCoolingTime(NeedToChangeStatus.SPEED_UP.value)
    time.sleep(5)
    scheduler.Delivered()
    assert scheduler.GetRemainDrug("B") == 5 and scheduler.GetRemainDrug("C") == 7
    time.sleep(2)
    assert scheduler.GetNeedToChangeStatus() == NeedToChangeStatus.SLOW_DOWN.value
    scheduler.GetNewRequest("C", 1)
    nextTarget, _ = scheduler.GetNextTarget()
    assert nextTarget["requestType"] == "A" and (
        nextTarget["deliverDestination"] == 4 or nextTarget["deliverDestination"] == 2
    )
    time.sleep(6)
    scheduler.DrugLoaded()
    assert scheduler.GetRemainDrug("B") == 8
    time.sleep(1)
    assert scheduler.GetNeedToChangeStatus() == NeedToChangeStatus.SLOW_DOWN.value
    scheduler.UpdateDrugCoolingTime(NeedToChangeStatus.SLOW_DOWN.value)
    time.sleep(8)
    assert scheduler.GetRemainDrug("B") == 10 and scheduler.GetRemainDrug("C") == 13

    # scheduler.GetNewRequest("A", 1)
    # nextTarget, _ = scheduler.GetNextTarget()
    # needToChange = scheduler.GetNeedToChangeStatus()
    # assert needToChange == NeedToChangeStatus.DONT_CHANGE.value
    # scheduler.DrugLoaded()
    # time.sleep(5)
    # scheduler.Delivered()
    # scheduler.GetNewRequest("B", 2)
    # scheduler.GetNewRequest("C", 4)
    # scheduler.GetNewRequest("A", 1)
    # nextTarget, _ = scheduler.GetNextTarget()
    # # 药物A无剩余，此时应该取药物B
    # assert nextTarget["requestType"] == "B" and nextTarget["deliverDestination"] == 2

    # needToChange = scheduler.GetNeedToChangeStatus()
    # assert needToChange == NeedToChangeStatus.SPEED_UP.value
    # time.sleep(1)
    # assert scheduler.UpdateDrugCoolingTime(NeedToChangeStatus.SPEED_UP.value)
    # # 等待药物A补充，同时送达当前药物
    # scheduler.Delivered()
    # scheduler.GetNewRequest("B", 3)
    # nextTarget, _ = scheduler.GetNextTarget()
    # assert nextTarget["requestType"] == "A" and nextTarget["deliverDestination"] == 1
    # scheduler.DrugLoaded()
    scheduler.Terminate()
    time.sleep(1)
    print("通过测试!")
    exit(0)

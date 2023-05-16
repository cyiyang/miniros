# -*- coding: utf-8 -*-
"""多车回归测试，修改后的Scheduler应通过该测试
"""

import sys

sys.path.append("..")

import os

os.chdir(os.path.dirname(__file__))
import time

from scheduler import Scheduler, NeedToChangeStatus

scheduler = Scheduler()
scheduler.GetNewRequest("A", 1)
scheduler.GetNewRequest("B", 2)
scheduler.GetNewRequest("C", 3)
scheduler.GetNewRequest("A", 4)

car0Target = scheduler.GetNextTarget(0)
car1Target = scheduler.GetNextTarget(1)

print(car0Target)
print(car1Target)

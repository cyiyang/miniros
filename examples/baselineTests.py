"""基线测试，修改后的Scheduler应通过该测试
"""

import sys

sys.path.append("..")

import os

os.chdir(os.path.dirname(__file__))
import time

from scheduler import Scheduler

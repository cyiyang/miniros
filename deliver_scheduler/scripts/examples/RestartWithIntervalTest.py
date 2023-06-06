import sys

sys.path.append("/home/ncut/drug-deliverer/deliver_scheduler/scripts")
import time

from scheduler import ReloadableTimer


def SayHi():
    print("Hi!")


timer = ReloadableTimer(5, True, SayHi)
timer.restartWithInterval(10)
timer.start()
time.sleep(15)

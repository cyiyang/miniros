import sys

sys.path.append("/home/ncut/drug-deliverer/deliver_scheduler/scripts")
import time

from scheduler import ReloadableTimer


def timeCostlyFun():
    print("timeCostlyFun running!")
    time.sleep(5)
    print("timeCostlyFun finished!")


def MakePrint():
    print("Triggered!")


if __name__ == "__main__":
    # timer = ReloadableTimer(5, True, timeCostlyFun)
    timer = ReloadableTimer(5, True, MakePrint)
    timer.start()
    print(timer.getElapsedTime())
    timer.setRemainTime(1)
    print(timer.getElapsedTime())
    print(timer.getRemainTime())
    while True:
        pass

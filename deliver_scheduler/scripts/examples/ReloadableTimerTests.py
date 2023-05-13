import sys

sys.path.append("/home/ncut/drug-deliverer/deliver_scheduler/scripts")
import time

from scheduler import ReloadableTimer


def timeCostlyFun():
    print("timeCostlyFun running!")
    time.sleep(5)
    print("timeCostlyFun finished!")


if __name__ == "__main__":
    timer = ReloadableTimer(5, True, timeCostlyFun)
    timer.start()
    while True:
        pass

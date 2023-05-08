#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os
import threading
import time

from playsound import playsound


class Announcer:
    def __init__(self) :
        self.DISPENSING_POINT_AUDIO = "/home/EPRobot/Music/dispense.mp3"
        self.PICK_UP_POINT_AUDIO = "/home/EPRobot/Music/pick_up.mp3"
        if (not os.path.exists(self.DISPENSING_POINT_AUDIO)) or (not os.path.exists(
            self.PICK_UP_POINT_AUDIO
        )):
            raise IOError("音频文件不存在")
        # dispensing point 为配药点音频, pick up point为取药点音频

    def arriveDispensingPoint(self):
        thread = threading.Thread(target=playsound, args=(self.DISPENSING_POINT_AUDIO,))
        thread.start()
        # thread.join()

    def arrivePickUpPoint(self):
        thread = threading.Thread(target=playsound, args=(self.PICK_UP_POINT_AUDIO,))
        thread.start()
        # thread.join()


if __name__ == "__main__":
    announcer = Announcer()
    announcer.arriveDispensingPoint()
    announcer.arrivePickUpPoint()


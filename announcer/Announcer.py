from playsound import playsound
import threading
import time


class Announcer:
    def __init__(self) -> None:
        self.DISPENSING_POINT_AUDIO = "announcer/到达配药点.mp3"
        self.PICK_UP_POINT_AUDIO = "announcer/到达取药点.mp3"
        # dispensing point 为配药点音频, pick up point为取药点音频

    def arriveDispensingPoint(self):
        thread = threading.Thread(target=playsound, args=(self.DISPENSING_POINT_AUDIO,))
        thread.start()
        thread.join()

    def arrivePickUpPoint(self):
        thread = threading.Thread(target=playsound, args=(self.PICK_UP_POINT_AUDIO,))
        thread.start()
        thread.join()


if __name__ == "__main__":
    announcer = Announcer()
    announcer.arriveDispensingPoint()
    announcer.arrivePickUpPoint()

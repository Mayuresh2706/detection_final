from gpiozero import Servo
import time

# Calibrated: 90° at full speed = 0.43s
DEG_PER_SEC = 90 / 0.40
SHOT_PAUSE  = 0.2


class MG996R:
    def __init__(self, pin=18):
        self.servo = Servo(pin,
                           min_pulse_width=1/1000,   # 1ms
                           max_pulse_width=2/1000)   # 2ms
        self.servo.value = None  # stop

    def fire(self, count=3):
        for i in range(count):
            self.servo.value = 1  # full CCW
            time.sleep(360 / DEG_PER_SEC)
            self.servo.value = None   # stop
            if i < count - 1:
                time.sleep(SHOT_PAUSE)

    def close(self):
        self.servo.value = None
        self.servo.close()

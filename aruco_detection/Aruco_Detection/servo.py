import lgpio
import time

# RPi5 uses gpiochip4
GPIOCHIP = 4

# Pulse widths in microseconds — calibrated from ESP32 values
# ESP32 duty/1023 * 20000us = pulse width
STOP_US     = 1505   # duty=77  → 1.5ms stop
CCW_US      = 2014   # duty=103 → 2.0ms full speed CCW

FREQ        = 50
DEG_PER_SEC = 90 / 0.43  # calibrated
SHOT_PAUSE  = 0.2


class MG996R:
    def __init__(self, pin=18):
        self.pin = pin
        self.chip = lgpio.gpiochip_open(GPIOCHIP)
        lgpio.tx_servo(self.chip, self.pin, STOP_US, FREQ)

    def fire(self, count=3):
        for i in range(count):
            lgpio.tx_servo(self.chip, self.pin, CCW_US, FREQ)
            time.sleep(90 / DEG_PER_SEC)
            lgpio.tx_servo(self.chip, self.pin, STOP_US, FREQ)
            if i < count - 1:
                time.sleep(SHOT_PAUSE)

    def close(self):
        lgpio.tx_servo(self.chip, self.pin, 0)
        lgpio.gpiochip_close(self.chip)

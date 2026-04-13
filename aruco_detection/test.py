import time

from servo import MG996R

servo = MG996R(pin=18)
try:
    servo.fire(count=1)
    time.sleep(5)
    servo.fire(count=1)
    time.sleep(5)
    servo.fire(count=1)
finally:
    servo.close()

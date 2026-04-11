

from servo import MG996R

servo = MG996R(pin=12)
try:
    servo.fire(count=3)
finally:
    servo.close()

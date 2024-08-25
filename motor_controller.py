from gpiozero import Motor
import time

motor = Motor(17, 18)

delay = 2

while True:
    motor.stop()
    print("The motor is stopped")
    time.sleep(delay)
    motor.forward()
    print("The motor is going forward")
    time.sleep(delay)
    motor.stop()
    print("The motor is stopped")
    time.sleep(delay)
    motor.backward()
    print("The motor is going backwards")
    time.sleep(delay)

import time
import math
from board import SCL, SDA
import busio

# Import the PCA9685 module. Available in the bundle and here:
#   https://github.com/adafruit/Adafruit_CircuitPython_PCA9685
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(SCL, SDA)

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c)
# You can optionally provide a finer tuned reference clock speed to improve the accuracy of the
# timing pulses. This calibration will be specific to each board and its environment. See the
# calibration.py example in the PCA9685 driver.
# pca = PCA9685(i2c, reference_clock_speed=25630710)
pca.frequency = 50
servo1 = servo.Servo(pca.channels[1])
servo2 = servo.Servo(pca.channels[2])
servo3 = servo.Servo(pca.channels[3])
servo4 = servo.Servo(pca.channels[4])
servo5 = servo.Servo(pca.channels[5])
# The pulse range is 750 - 2250 by default. This range typically gives 135 degrees of
# range, but the default is to use 180 degrees. You can specify the expected range if you wish:

def rotate_servo(servo, target_angle, delay=0.05):
    if servo == 1:
        for angle in range(abs(target_angle-10),target_angle):
            servo1.angle = angle
            time.sleep(delay)
        servo1.angle = target_angle
    elif servo == 2:
        for angle in range(abs(target_angle-10),target_angle):
            servo2.angle = angle
            time.sleep(delay)
        servo2.angle = target_angle
    elif servo == 3:
        for angle in range(abs(target_angle-10),target_angle):
            servo3.angle = angle
            time.sleep(delay)
        servo3.angle = target_angle
    elif servo == 4:
        for angle in range(abs(target_angle-10),target_angle):
            servo4.angle = angle
            time.sleep(delay)
        servo4.angle = target_angle
    elif servo == 5:
        for angle in range(abs(target_angle-10),target_angle):
            servo5.angle = angle
            time.sleep(delay)
        servo5.angle = target_angle
def main():
    rotate_servo(5, 45)
    rotate_servo(1, 10)
    rotate_servo(2, 70)
    rotate_servo(3, 180)
    rotate_servo(4, 90)
    rotate_servo(5, 75)
    time.sleep(1)
    rotate_servo(3, 90)
    time.sleep(0.5)
    rotate_servo(2, 90)
    time.sleep(0.5)
    rotate_servo(1, 90)
if __name__ == '__main__':
    main()
    pca.deinit()


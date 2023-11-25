import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

GPIO_TRIG = 20
GPIO_ECHO = 21

GPIO.setup(GPIO_TRIG, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

def distance():
    # 发送高电平信号到 Trig 引脚
    GPIO.output(GPIO_TRIG, 0)  # 开始为低电平
    time.sleep(0.000002)
    GPIO.output(GPIO_TRIG, 1)

    # 持续 10 us
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIG, 0)

    # 高电平持续时间就是超声波从发射到返回的时间
    while GPIO.input(GPIO_ECHO) == GPIO.LOW: pass
    start_time = time.time()
    while GPIO.input(GPIO_ECHO) == GPIO.HIGH: pass
    stop_time = time.time()

    # 计算距离 声波的速度为 34000cm/s。
    distance = ((stop_time - start_time) * 34000) / 2

    return distance

while True:
    dist = distance()
    print("距离：{:.2f} cm".format(dist))
    time.sleep(1)


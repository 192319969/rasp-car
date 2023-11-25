import cv2
import numpy as np
import time
import RPi.GPIO as GPIO
from datetime import datetime

# 定义方向
FORWARD = 1
BACKWARD = 2
STOP = 3

# 定义引脚
pins = {

    "back_left_in1": 7,
    "back_left_in2": 11,

    "back_right_in1": 15,
    "back_right_in2": 13,

    "back_left_en": 12,
    "back_right_en": 16,

    "front_left_in1": 29,
    "front_left_in2": 31,

    "front_right_in1": 33,
    "front_right_in2": 37,

    "front_left_en": 22,
    "front_right_en": 32
}

left_motors = {
    "back_left",
    "front_left"
}

right_motors = {
    "back_right",
    "front_right"
}

all_motors = {
    "back_left",
    "front_left",
    "back_right",
    "front_right"
}

pwms = {
    "back_left_pwm": None,
    "back_right_pwm": None,
    "front_left_pwm": None,
    "front_right_pwm": None
}


# 初始化GPIO
def setup_gpio():
    GPIO.setmode(GPIO.BOARD)
    for pin in pins.values():
        GPIO.setup(pin, GPIO.OUT)


# 初始化PWM
def setup_pwm():
    for motor in all_motors:
        pwm_pin = pins[f"{motor}_en"]
        pwms[f"{motor}_pwm"] = GPIO.PWM(pwm_pin, 100)
        pwms[f"{motor}_pwm"].start(0)


def set_motor_direct(direction, motor):
    if direction == FORWARD:
        GPIO.output(pins[f"{motor}_in1"], GPIO.HIGH)
        GPIO.output(pins[f"{motor}_in2"], GPIO.LOW)
    elif direction == BACKWARD:
        GPIO.output(pins[f"{motor}_in1"], GPIO.LOW)
        GPIO.output(pins[f"{motor}_in2"], GPIO.HIGH)
    elif direction == STOP:
        GPIO.output(pins[f"{motor}_in1"], GPIO.LOW)
        GPIO.output(pins[f"{motor}_in2"], GPIO.LOW)


def set_motor_speed(motors, speed):
    for motor in motors:
        pwms[f"{motor}_pwm"].ChangeDutyCycle(speed)


def clean_gpio():
    GPIO.cleanup()
    for motor in all_motors:
        pwms[f"{motor}_pwm"].stop()


def drive_PID(left_speed, right_speed, defTime):
    left_speed *= 100
    right_speed *= 100
    for motor in right_motors:
        set_motor_direct(FORWARD, motor)
    set_motor_speed(right_motors, right_speed)
    for motor in left_motors:
        set_motor_direct(FORWARD, motor)
    set_motor_speed(left_motors, left_speed)

    time.sleep(defTime)


# 初始化摄像头
camera = cv2.VideoCapture(0)

# 初始化速度
speedDef = 0.5
leftSpeed = speedDef
rightSpeed = speedDef
defTime = 0.01
driveTime = defTime
maxDiff = 0.2

# 目标球的坐标
ballX = 0.0
ballY = 0.0

# 初始化PID参数
kp = 1.0
ki = 1.0
kd = 1.0

x_PID = {'axis': 'X',
         'lastTime': int(round(time.time() * 1000)),
         'lastError': 0.0,
         'error': 0.0,
         'duration': 0.0,
         'sumError': 0.0,
         'dError': 0.0,
         'PID': 0.0}
y_PID = {'axis': 'Y',
         'lastTime': int(round(time.time() * 1000)),
         'lastError': 0.0,
         'error': 0.0,
         'duration': 0.0,
         'sumError': 0.0,
         'dError': 0.0,
         'PID': 0.0}


def PID(axis):
    lastTime = axis['lastTime']
    lastError = axis['lastError']

    now = int(round(time.time() * 1000))
    duration = now - lastTime

    axis['sumError'] += axis['error'] * duration
    axis['dError'] = (axis['error'] - lastError) / duration

    if axis['sumError'] > 1:
        axis['sumError'] = 1

    if axis['sumError'] < -1:
        axis['sumError'] = -1

    axis['PID'] = kp * axis['error'] + ki * axis['sumError'] + kd * axis['dError']

    axis['lastError'] = axis['error']
    axis['lastTime'] = now

    return axis


setup_gpio()
setup_pwm()

status_file=None

# 主循环之前初始化文件对象
def initialize_status_file():
    global status_file
    status_file = open('status.log', 'a')

# 主循环结束后关闭文件对象
def close_status_file():
    global status_file
    if status_file:
        status_file.close()

while True:
    # 读取摄像头帧
    ret, frame = camera.read()

    # 获取图像的中心点坐标
    height, width, _ = frame.shape  # 获取图像的高度和宽度
    xMid = width / 2 * 1.0  # 计算图像中心点的 x 坐标
    yMid = height / 2 * 1.0  # 计算图像中心点的 y 坐标

    # 将帧转换为HSV色彩空间
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 定义网球的颜色范围（在HSV色彩空间中）
    lower_color = np.array([27, 104, 25])
    upper_color = np.array([77, 255, 255])

    # 根据颜色范围创建掩膜
    mask = cv2.inRange(hsv, lower_color, upper_color)

    # 对掩膜进行形态学操作（可选，用于去除噪点）
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    # 进行霍夫圆检测
    circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, dp=1, minDist=1000, param1=50, param2=15, minRadius=50,maxRadius=150)

    # 绘制检测到的圆
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:
            cv2.circle(frame, (x, y), r, (0, 255, 0), 4)

            ballX = int(x)
            ballY = int(y)
            #print("now:", datetime.now().strftime("%H:%M:%S.%f")[:-3], "ballX=", ballX, "ballY=", ballY)

            xVariance = (ballX - xMid) / xMid
            yVariance = (ballY - yMid) / yMid
            x_PID['error'] = xVariance / xMid
            y_PID['error'] = yVariance / yMid
            x_PID = PID(x_PID)
            y_PID = PID(y_PID)

            leftSpeed = (speedDef * y_PID['PID']) + (maxDiff * x_PID['PID'])
            rightSpeed = (speedDef * y_PID['PID']) - (maxDiff * x_PID['PID'])

            if leftSpeed > (speedDef + maxDiff):
                leftSpeed = (speedDef + maxDiff)
            if leftSpeed < -(speedDef + maxDiff):
                leftSpeed = -(speedDef + maxDiff)
            if rightSpeed > (speedDef + maxDiff):
                rightSpeed = (speedDef + maxDiff)
            if rightSpeed < -(speedDef + maxDiff):
                rightSpeed = -(speedDef + maxDiff)

            #write(log)
            drive_PID(leftSpeed, rightSpeed, driveTime)

    # 显示结果图像

    res = cv2.bitwise_and(frame, frame, mask=mask)

    #cv2.imshow('mask1', mask)

    cv2.imshow('res', res)
    cv2.imshow("Frame", frame)

    # 按下 'q' 键退出循环
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放摄像头资源并关闭窗口
clean_gpio()
camera.release()
cv2.destroyAllWindows()

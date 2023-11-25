from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
from adafruit_motorkit import MotorKit

kit = MotorKit()
motors = [kit.motor3, kit.motor4, kit.motor1, kit.motor2]
motorSpeed = [0, 0, 0, 0]
speedDef = 0.5
leftSpeed = speedDef
rightSpeed = speedDef
defTime = 0.01
driveTime = defTime
maxDiff = 0.2

kp = 1.0
ki = 1.0
kd = 1.0
ballX = 0.0
ballY = 0.0

x = {'axis': 'X',
     'lastTime': int(round(time.time() * 1000)),
     'lastError': 0.0,
     'error': 0.0,
     'duration': 0.0,
     'sumError': 0.0,
     'dError': 0.0,
     'PID': 0.0}
y = {'axis': 'Y',
     'lastTime': int(round(time.time() * 1000)),
     'lastError': 0.0,
     'error': 0.0,
     'duration': 0.0,
     'sumError': 0.0,
     'dError': 0.0,
     'PID': 0.0}


def driveMotors(leftChnl=speedDef, rightChnl=speedDef,duration=defTime):
    motorSpeed[0] = leftChnl
    motorSpeed[1] = leftChnl
    motorSpeed[2] = rightChnl
    motorSpeed[3] = rightChnl

    motors[0].throttle = motorSpeed[0]
    motors[1].throttle = motorSpeed[1]
    motors[2].throttle = motorSpeed[2]
    motors[3].throttle = motorSpeed[3]


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


def killMotors():
    motors[0].throttle = 0.0
    motors[1].throttle = 0.0
    motors[2].throttle = 0.0
    motors[3].throttle = 0.0


params = cv2.SimpleBlobDetector_Params()
params.filterByColor = False
params.filterByArea = True
params.minArea = 2500
params.maxArea = 30000
params.filterByInertia = False
params.filterByConvexity = False
params.filterByCircularity = True
params.minCircularity = 0.4
params.maxCircularity = 1
det = cv2.SimpleBlobDetector_create(params)
# set red filter range
lower_red = np.array([160, 60, 20])  # 80 for blue
upper_red = np.array([180, 255, 255])  # 130 for blue

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
time.sleep(0.1)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array
    height, width, chan = np.shape(image)
    xMid = width / 2 * 1.0
    yMid = height / 2 * 1.0
    imgHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    redMask = cv2.inRange(imgHSV, lower_red, upper_red)
    blur = cv2.blur(redMask, (10, 10))
    res = cv2.bitwise_and(image, image, mask=blur)
    keypoints = det.detect(blur)
    try:
        ballX = int(keypoints[0].pt[0])
        ballY = int(keypoints[0].pt[1])
    except:
        pass

    # not necessary if you dont need to see camera input via VNC
    cv2.drawKeypoints(image, keypoints, image, (0, 255, 0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)


    xVariance = (ballX - xMid) / xMid
    yVariance = (ballY - yMid) / yMid
    x['error'] = xVariance / xMid
    y['error'] = yVariance / yMid
    x = PID(x)
    y = PID(y)

    leftSpeed = (speedDef * y['PID']) + (maxDiff * x['PID'])
    rightSpeed = (speedDef * y['PID']) - (maxDiff * x['PID'])
    if leftSpeed > (speedDef + maxDiff):
        leftSpeed = (speedDef + maxDiff)
    if leftSpeed < -(speedDef + maxDiff):
        leftSpeed = -(speedDef + maxDiff)
    if rightSpeed > (speedDef + maxDiff):
        rightSpeed = (speedDef + maxDiff)
    if rightSpeed < -(speedDef + maxDiff):
        rightSpeed = -(speedDef + maxDiff)



    driveMotors(leftSpeed, rightSpeed, driveTime)


    # also not necessary
    cv2.imshow("Frame", image)
    # cv2.imshow("Mask", blur)   <- shows the mask frame
    key = cv2.waitKey(1) & 0xFF
    rawCapture.truncate(0)
    if key == ord("q"):
        break

killMotors()
cv2.destroyAllWindows()
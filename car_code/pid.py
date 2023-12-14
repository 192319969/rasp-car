import time
# 初始化速度
speedDef = 0.2
leftSpeed = speedDef
rightSpeed = speedDef
defTime = 0.01

# 目标球的坐标
ballX = 0.0

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
    

def get_ball_pos(x1, x2):
    global ballX
    ballX = int((x1 + x2) / 2)


def is_at_center(xMid):
    if (ballX <= xMid + 70) and (ballX >= xMid -70):
        return True
    else:
        return False


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


def cal_speed():
    global leftSpeed, rightSpeed
    # leftSpeed = maxDiff * x_PID['PID']
    # rightSpeed = maxDiff * x_PID['PID']
    #
    # if leftSpeed > maxDiff:
    #     leftSpeed = maxDiff
    # if leftSpeed < -maxDiff:
    #     leftSpeed = -maxDiff
    # if rightSpeed > maxDiff:
    #     rightSpeed = maxDiff
    # if rightSpeed < -maxDiff:
    #     rightSpeed = -maxDiff

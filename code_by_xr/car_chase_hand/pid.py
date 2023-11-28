import time

# 初始化速度
speedDef = 0.2
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

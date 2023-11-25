import RPi.GPIO as GPIO
import time

# 定义速度
SPEED_FORWARD = 100
SPEED_TURN = 50

# 定义方向
FORWARD = 1
BACKWARD = 2
STOP = 3

# 定义时间
defTime = 1

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

factor = {
    "back_left": 0.75,
    "front_left": 0.75,
    "back_right": 1,
    "front_right": 0.75
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


def drive_PID(left_speed, right_speed, time=defTime):
    left_speed *= 100
    right_speed *= 100
    for motor in right_motors:
        set_motor_direct(FORWARD, motor)
    set_motor_speed(right_motors, right_speed)
    for motor in left_motors:
        set_motor_direct(FORWARD, motor)
    set_motor_speed(left_motors, left_speed)

    time.sleep(time)


# 设置电机速度
def set_motor_speed(motors, speed):
    for motor in motors:
        pwms[f"{motor}_pwm"].ChangeDutyCycle(speed)


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


# 控制小车前进
def car_move_forward():
    for motor in all_motors:
        set_motor_direct(FORWARD, motor)
    set_motor_speed(all_motors, SPEED_FORWARD)


# 控制小车后退
def car_move_backward():
    for motor in all_motors:
        set_motor_direct(BACKWARD, motor)
    set_motor_speed(all_motors, SPEED_FORWARD)


# 控制小车左前转
def car_front_left(angle=0):
    for motor in right_motors:
        set_motor_direct(FORWARD, motor)
    set_motor_speed(right_motors, SPEED_FORWARD)
    for motor in left_motors:
        set_motor_direct(BACKWARD, motor)
    set_motor_speed(left_motors, SPEED_TURN)
    time.sleep((angle / 60) * factor["front_left"])


# 控制小车左后转
def car_back_left(angle=0):
    for motor in right_motors:
        set_motor_direct(BACKWARD, motor)
    set_motor_speed(right_motors, SPEED_FORWARD)
    for motor in left_motors:
        set_motor_direct(FORWARD, motor)
    set_motor_speed(left_motors, SPEED_TURN)
    time.sleep((angle / 60) * factor["back_left"])


# 控制小车右前转
def car_front_right(angle=0):
    for motor in left_motors:
        set_motor_direct(FORWARD, motor)
    set_motor_speed(left_motors, SPEED_FORWARD)
    for motor in right_motors:
        set_motor_direct(BACKWARD, motor)
    set_motor_speed(right_motors, SPEED_TURN)
    time.sleep((angle / 60) * factor["front_right"])


# 控制小车后前转
def car_back_right(angle=0):
    for motor in left_motors:
        set_motor_direct(BACKWARD, motor)
    set_motor_speed(left_motors, SPEED_FORWARD)
    for motor in right_motors:
        set_motor_direct(FORWARD, motor)
    set_motor_speed(right_motors, SPEED_TURN)
    time.sleep((angle / 60) * factor["back_right"])


# 停止小车
def car_stop():
    for motor in all_motors:
        set_motor_direct(STOP, motor)
    set_motor_speed(all_motors, 0)
    time.sleep(0.5)


# 清理GPIO
def clean_gpio():
    GPIO.cleanup()
    for motor in all_motors:
        pwms[f"{motor}_pwm"].stop()


def car_initial():
    setup_gpio()
    setup_pwm()


def car_exit():
    car_stop()
    clean_gpio()


if __name__ == '__main__':
    car_initial()

    car_move_backward()
    time.sleep(2)

    car_stop()

    car_back_right(90)

    car_stop()

    car_move_backward()
    time.sleep(2)

    car_exit()

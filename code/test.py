import car
import time


if __name__ == "__main__":
    car.initial()

    car.front_left(15)
    time.sleep(car.def_TIME)

    car.exit()
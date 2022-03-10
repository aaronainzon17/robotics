#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
from tokenize import Double
import numpy as np
import time
import math
from Robot import Robot


def dos_puntos(robot, a, d, dist):
    r2 = math.sqrt(dist**2 + (d-a)**2)
    th = np.rad2deg(math.acos((dist**2 + r2**2 - (d-a)**2)/(2*dist*r2)))

    v = 200
    w = 90
    w1 = np.rad2deg((float)(v/a))
    w2 = np.rad2deg((float)(v/d))

    robot.setSpeed(0, w)    # gira 90 grados a la izquierda
    time.sleep(90/w)
    robot.setSpeed(v, -w1)  # cuarto de circunferencia a la derecha
    time.sleep(90/w1)
    robot.setSpeed(0, 0)
    time.sleep(0.5)
    robot.setSpeed(0, w)    # gira th grados a la izquierda
    time.sleep(th/w)
    robot.setSpeed(v, 0)    # avanza r2 mm en linea recta
    time.sleep(r2/v)
    robot.setSpeed(0, -w)   # gira th grados a la derecha
    time.sleep(th/w)
    robot.setSpeed(v, -w2)  # media circunferencia a la derecha
    time.sleep(180/w2)
    robot.setSpeed(0, -w)   # gira th grados a la derecha
    time.sleep(th/w)
    robot.setSpeed(v, 0)    # avanza r2 mm en linea recta
    time.sleep(r2/v)
    robot.setSpeed(0, w)    # gira th grados a la izquierda
    time.sleep(th/w)
    robot.setSpeed(v, -w1)  # cuarto de circunferencia a la derecha
    time.sleep(90/w1)
    robot.setSpeed(0, 0)


def ocho(robot, d):
    v = 200
    w = np.rad2deg((float)(v/d))

    robot.setSpeed(0, -90)
    time.sleep(1)
    robot.setSpeed(v, w)
    time.sleep(180/w)
    robot.setSpeed(v, -w)
    time.sleep(360/w)
    robot.setSpeed(v, w)
    time.sleep(180/w)
    robot.setSpeed(0, 0)


def main(args):
    try:
        if args.radioD < 0:
            print('d must be a positive value')
            exit(1)

        # Instantiate Odometry. Default value will be 0,0,0
        # robot = Robot(init_position=args.pos_ini)
        robot = Robot()

        print("X value at the beginning from main X= %.2f" % (robot.x.value))

        # 1. launch updateOdometry Process()
        robot.startOdometry()

        # 2. perform trajectory

        # RECTANGLE
        print("Start : %s" % time.ctime())

        # robot.setSpeed(200, 0)
        # time.sleep(4)
        # robot.setSpeed(0, 90)
        # time.sleep(1)
        # robot.setSpeed(200, 0)
        # time.sleep(2)
        # robot.setSpeed(0, 90)
        # time.sleep(1)
        # robot.setSpeed(200, 0)
        # time.sleep(4)
        # robot.setSpeed(0, 90)
        # time.sleep(1)
        # robot.setSpeed(200, 0)
        # time.sleep(2)
        # robot.setSpeed(0, 90)
        # time.sleep(1)

        #ocho(robot, 400)

        dos_puntos(robot, 200, 400, 800)

        print("End : %s" % time.ctime())

        robot.lock_odometry.acquire()
        print("Odom values at main at the END: %.2f, %.2f, %.2f " %
              (robot.x.value, robot.y.value, robot.th.value))
        robot.lock_odometry.release()

        # DUMMY CODE! delete when you have your own
        # robot.setSpeed(0.25,0)
        #print("Start : %s" % time.ctime())
        # time.sleep(3)
        #print("X value from main tmp %d" % robot.x.value)
        # time.sleep(3)
        #print("End : %s" % time.ctime())

        # robot.lock_odometry.acquire()
        #print("Odom values at main at the END: %.2f, %.2f, %.2f " % (robot.x.value, robot.y.value, robot.th.value))
        # robot.lock_odometry.release()

        # PART 1:
        # robot.setSpeed()
        # until ...

        # PART 2:
        # robot.setSpeed()
        # until ...

        # ...

        # 3. wrap up and close stuff ...
        # This currently unconfigure the sensors, disable the motors,
        # and restore the LED to the control of the BrickPi3 firmware.
        robot.stopOdometry()

    except KeyboardInterrupt:
        # except the program gets interrupted by Ctrl+C on the keyboard.
        # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        robot.stopOdometry()


if __name__ == "__main__":

    # get and parse arguments passed to main
    # Add as many args as you need ...
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--radioD", help="Radio to perform the 8-trajectory (mm)",
                        type=float, default=40.0)
    args = parser.parse_args()

    main(args)

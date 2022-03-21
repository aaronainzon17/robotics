#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
from tokenize import Double
import numpy as np
import time
import math
from Robot import Robot

"""Trayectoria de dos circulos con tangentes con tiempos"""
def dos_puntos_time(robot, a, d, dist):
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

"""Trayectoria del ocho definida con tiempos"""
def ocho_time(robot, d):
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

"""Trayectoria del rectangulo definida con tiempos"""
def rectangulo_time(robot):
    robot.setSpeed(200, 0)
    time.sleep(4)
    robot.setSpeed(0, 60)
    time.sleep(1.5)
    robot.setSpeed(200, 0)
    time.sleep(2)
    robot.setSpeed(0, 60)
    time.sleep(1.5)
    robot.setSpeed(200, 0)
    time.sleep(4)
    robot.setSpeed(0, 60)
    time.sleep(1.5)
    robot.setSpeed(200, 0)
    time.sleep(2)
    robot.setSpeed(0, 60)
    time.sleep(1.5)

def normalizar(th):
    if th > math.pi:
        th = th - 2 * math.pi
    elif th < -math.pi:
        th = th + 2 * math.pi
    return th

def check_position(robot,x,y,th,x_err,y_err, angular_err):
        [x_now,y_now,th_now] = robot.readOdometry()
        reached = False
        #if abs(x-x_now) <= pos_err & abs(y-y_now) <= pos_err & abs(th-th_now) <= angular_err: 
        #    reached = 0 
        #else: 
        #    reached = 1
        while not reached:
            print("-------------------------------------------")
            print("quiero llegar a ",x," ", y, " ",th)
            print("estoy en ",x_now, " " , y_now, " ", th_now)
            print("-------------------------------------------")
            #if x_now > x:
                #print("ERROR: no ha parado y se ha superado el umbral")
                #robot.setSpeed(0,0)
               
            if (abs(abs(x)-abs(x_now)) <= x_err) and (abs(abs(y)-abs(y_now)) <= y_err) and (abs(abs(th)-abs(th_now)) <= angular_err):
            #if (abs(x-x_now) <= pos_err) or (abs(y-y_now) <= pos_err) or (abs(th-th_now) <= angular_err):  
                reached = True
                print("Se ha alcanzado el punto:[",x_now,",",y_now,",",th_now,"]")
            else: 
                [x_now,y_now,th_now] = robot.readOdometry()
                print("La posicion actual es:",x_now,y_now)

            time.sleep(robot.getPeriod())
            


def rectangulo(robot, base, altura): 
    # robot.setSpeed(150, 0)
    # check_position(robot,base, 0, 0, 4, 100, np.deg2rad(20))
    
    
    robot.setSpeed(0, 45)
    check_position(robot, base, 0, normalizar(np.deg2rad(90)), 100, 100, np.deg2rad(0.3))
    
    # robot.setSpeed(150, 0)
    # check_position(robot,base, altura, normalizar(np.deg2rad(90)), 100, 4, 100)

    # robot.setSpeed(0, 45)
    # check_position(robot,base, altura, normalizar(np.deg2rad(180)),100, 100, np.deg2rad(1))

    # robot.setSpeed(150, 0)
    # check_position(robot,0, altura, normalizar(np.deg2rad(180)), 100, 25, np.deg2rad(90))   #tercera recta

    # robot.setSpeed(0, 45)
    # check_position(robot,0, altura, normalizar(np.deg2rad(270)), 100, 100, np.deg2rad(2))

    # robot.setSpeed(150, 0)
    # check_position(robot,0, 0, normalizar(np.deg2rad(270)), 30, 100, np.deg2rad(90))

    # robot.setSpeed(0, 45)
    # check_position(robot,0, 0, normalizar(np.deg2rad(0)), 100, 100, np.deg2rad(4))

    robot.setSpeed(0, 0)

"""def ocho(robot,primera_parada_x,primera_parada_y,segunda_parada_x,segunda_parada_y):
    robot.setSpeed(100, 20)
    check_position(robot,primera_parada_x, primera_parada_y, normalizar(np.deg2rad(180)), 30, np.deg2rad(5))
    
    
    robot.setSpeed(100, -20)
    check_position(robot, segunda_parada_x, segunda_parada_y, 0, 30, np.deg2rad(5))
    
    robot.setSpeed(100, -20)
    check_position(robot,primera_parada_x, segunda_parada_y, normalizar(np.deg2rad(180)), 30, np.deg2rad(5))

    robot.setSpeed(100, 20)
    check_position(robot,0, 0, 0, 30, np.deg2rad(5))

    robot.setSpeed(0, 0)
"""


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

        
        rectangulo(robot, 800,400)
        #ocho(robot,400,0,800,0)

        #ocho_time(robot, 400)
        #robot.setSpeed(0, 0)
        #dos_puntos_time(robot, 200, 400, 800)

        # robot.setSpeed(50,0)
        # check_position(robot,200,0,0,5,0.2)

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

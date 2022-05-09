#!/usr/bin/python
# -*- coding: UTF-8 -*-

import sys
sys.path.append('../libraries')

import argparse
from tokenize import Double
import numpy as np
import time
import math
from Robot import Robot
from MapLib import Map2D

"""
            TRABAJO FINAL:
Autores:
    - Aaron Ibañez Espes 779088
    - Sergio Gabete Cesar 774631
    - Belen Gimeno Chueca 756425
    - Pablo Gancedo Alcalde 736839 
"""

#Funcion que normaliza el angulo entre -pi, pi
def norm_pi(th):
    if th > math.pi:
        th = th - 2 * math.pi
    elif th < -math.pi:
        th = th + 2 * math.pi
    return th

def hom(x):
    """ A partir de un punto crea la matriz en coordenadas homogeneas """
    x1 = x[0]
    y1 = x[1]
    th = x[2]

    T = np.array([
        [np.cos(th), - np.sin(th), x1],
        [np.sin(th), np.cos(th), y1],
        [0, 0, 1]])

    return T


def loc(T):
    """ A partir de una matriz calcula el punto """
    nx = T[0][0]
    ny = T[1][0]
    px = T[0][2]
    py = T[1][2]
    x = np.array([px, py, norm_pi(np.arctan2(ny, nx))])

    return x


def check_position(robot, x, y, th, x_err, y_err, angular_err):
    """ check_position es la funcion de control de localizacion
        En ella se comprueba la posicion real del robot leida de los
        motores y se comprueba si se encuentra en la posicion deseada
        permitiendo un cierto error. """

    # Se lee incialmente la posicion del robot
    [x_now, y_now, th_now] = robot.readOdometry()
    reached = False

    while not reached:
        # print("-------------------------------------------")
        #print("quiero llegar a ", x, " ", y, " ", th)
        #print("estoy en ", x_now, " ", y_now, " ", th_now)
        # print("-------------------------------------------")

        # Se calcula el angulo
        error_ang = abs(th-th_now)
        if error_ang > math.pi:
            error_ang = (2*math.pi) - error_ang

        # Se comprueba que la trayectoria se esta relaizando dentro del error permitido
        if (abs(x-x_now) <= x_err) and (abs(y-y_now) <= y_err) and (error_ang <= angular_err):
            reached = True
            print("Se ha alcanzado el punto:[",
                  x_now, ",", y_now, ",", th_now, "]")
        else:
            [x_now, y_now, th_now] = robot.readOdometry()
            #print("La posicion actual es:", x_now, y_now)
        # time.sleep(period)


def s_A(robot, vel):
    """ La funcion ocho realiza la trayectoria de s del mapa A basandose en
        la odometria para detener al robot y comenzar con el siguiente movimiento """

    r = 400
    v = vel
    w = np.rad2deg((float)(v/r))

    pos = [[1,4],[1,2]]
    for point_map in pos:
        point = [200+point_map[0]*400, 200+point_map[1]*400]
        # Se mueve el robot a la siguiente celda
        robot.go(point[0],point[1])
        print('Voy a ',point)

    robot.setSpeed(0, 0)  # Parar el robot

def s_B(robot, vel):
    """ La funcion ocho realiza la trayectoria de s del mapa A basandose en
        la odometria para detener al robot y comenzar con el siguiente movimiento """

    r = 400
    v = vel
    w = np.rad2deg((float)(v/r))

    pos = [5,6]
    robot.setSpeed(0, 45)  # Giro de 45 deg a la izquierda
    check_position(robot, 200 + 400 * pos[0], 200 + 400 * pos[1], normalizar(np.deg2rad(-45)),
                   np.Infinity, np.Infinity, np.deg2rad(2))

    pos = [5,4]
    robot.setSpeed(v, -w)  # Primera semicircunferencia
    check_position(robot, 200 + 400 * pos[0], 200 + 400 * pos[1], normalizar(np.deg2rad(-135)),
                   10, 5, np.deg2rad(10))

    pos = [5,2]
    robot.setSpeed(v, w)  # Segunda semicircunferencia
    check_position(robot, 200 + 400 * pos[0], 200 + 400 * pos[1], normalizar(np.deg2rad(-45)),
                   20, 5, np.deg2rad(10))

    pos = [5,2]
    robot.setSpeed(0, -45)  # Giro de 45 deg a la derecha
    check_position(robot, 200 + 400 * pos[0], 200 + 400 * pos[1], normalizar(np.deg2rad(-90)),
                   np.Infinity, np.Infinity, np.deg2rad(2))


    robot.setSpeed(0, 0)  # Parar el robot


def main(args):
    try:
        # 0. Se carga el mapa en función de la marca en el suelo
        if args.mapa == "A":
            ini = [1,6]
            map_file = "mapaA_CARRERA.txt"
        elif args.mapa == "B": 
            ini = [5,6]
            map_file = "mapaB_CARRERA.txt"
        else:
            print('Mapa desconocido, seleccione mapa A o B')
            exit(1)
        
        robot = Robot(init_position=[200+ini[0]*400, 200+ini[1]*400,np.deg2rad(-90)])

        print("X value at the beginning from main X= %.2f" % (robot.x.value))

        # 1. Se incia la odometria u el proceso update odometry
        robot.startOdometry()

        #myMap = Map2D(map_file)

        # 2. perform trajectory

        print("Start : %s" % time.ctime())

        # Trayectoria en s
        if args.mapa == "A":
            # R2D2
            s_A(robot, 150)
        else:
            # BB8
            s_B(robot, 150)

        print("End : %s" % time.ctime())

        robot.lock_odometry.acquire()
        print("Odom values at main at the END: %.2f, %.2f, %.2f " %
              (robot.x.value, robot.y.value, robot.th.value))
        robot.lock_odometry.release()

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
    parser.add_argument("-m", "--mapa", help="Mapa seleccionado (A o B)",
                        type=str, default="A")
    args = parser.parse_args()

    main(args)

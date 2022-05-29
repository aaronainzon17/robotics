#!/usr/bin/python
# -*- coding: UTF-8 -*-

import sys

from cv2 import solve
sys.path.append('./libraries')
import os
import cv2
import argparse
from tokenize import Double
import numpy as np
import time
import math
from libraries.Robot import Robot
from libraries.MapLib import Map2D
from libraries.SolveMap import solveMap, solve_relative_map
from libraries.BlobDetector import getRedBloobs,detect_red
from ficheros_codigo_auxiliar.sample_matching import find_template
   
def s_A_ocho(robot, vel):
    """ La funcion ocho realiza la trayectoria de s del mapa A basandose en
        la odometria para detener al robot y comenzar con el siguiente movimiento """

    # Se mueve al centro de la primera celda
    #robot.go(600,2600, vel)
    robot.setSpeed(vel, 0)  # Giro de 45 deg a la derecha
    robot.check_position_3_values(600,2600, np.deg2rad(-90),np.Infinity, 5, np.deg2rad(10))
    # Comienza el circulo
    r = 400
    v = vel
    w = np.rad2deg((float)(v/r))

    pos = [1,6]
    robot.setSpeed(0, -w)  # Giro de 45 deg a la derecha
    robot.check_position_3_values(200 + 400 * pos[0], 200 + 400 * pos[1], np.deg2rad(-180), np.Infinity, np.Infinity, np.deg2rad(2))

    pos = [1,4]
    robot.setSpeed(v, w)  # Primera semicircunferencia
    robot.check_position_3_values(200 + 400 * pos[0], 200 + 400 * pos[1], np.deg2rad(0), 10, np.Infinity, np.deg2rad(10))
    
    pos = [1,2]
    robot.setSpeed(v, -w)  # Segunda semicircunferencia
    robot.check_position_3_values(200 + 400 * pos[0], 200 + 400 * pos[1], np.deg2rad(-180), 10, 400, np.deg2rad(10))
    
    robot.relocate()

    #pos = [1,2]
    #robot.setSpeed(0, 30)  # Giro de 45 deg a la izquierda
    #robot.check_position_3_values(200 + 400 * pos[0], 200 + 400 * pos[1], np.deg2rad(-90), np.Infinity, np.Infinity, np.deg2rad(2))


    robot.setSpeed(0, 0)  # Parar el robot    

def s_B(robot, vel):
    """ La funcion s_B realiza la trayectoria de s del mapa B basandose en
        la odometria para detener al robot y comenzar con el siguiente movimiento """  
    
    pos = [[6,5],[5,4],[4,3],[5,1.8]]
    for point_map in pos:
        point = [200+point_map[0]*400, 200+point_map[1]*400]
        # Se mueve el robot a la siguiente celda
        robot.go(point[0],point[1],vel)
        print('Voy a ',point)
    robot.align(1800,0,np.deg2rad(1))

def s_B_ocho(robot, vel):
    """ La funcion ocho realiza la trayectoria de s del mapa B basandose en
        la odometria para detener al robot y comenzar con el siguiente movimiento """

    # Se mueve al centro de la primera celda
    #robot.go(600,2600, vel)
    robot.setSpeed(vel, 0)
    robot.check_position_3_values(2200,2600, np.deg2rad(-90),10, 5, np.deg2rad(2))
    # Comienza el circulo
    r = 400
    v = vel
    w = np.rad2deg((float)(v/r))

    pos = [5,6]
    robot.setSpeed(0, w)  # Giro de 45 deg a la izquierda
    robot.check_position_3_values(200 + 400 * pos[0], 200 + 400 * pos[1], np.deg2rad(0), np.Infinity, np.Infinity, np.deg2rad(2))

    pos = [5,4]
    robot.setSpeed(v, w)  # Primera semicircunferencia
    robot.check_position_3_values(200 + 400 * pos[0], 200 + 400 * pos[1], np.deg2rad(180), 10, np.Infinity, np.deg2rad(10))
    
    pos = [5,2]
    robot.setSpeed(v, -w)  # Segunda semicircunferencia
    robot.check_position_3_values(200 + 400 * pos[0], 200 + 400 * pos[1], np.deg2rad(0), 10, 400, np.deg2rad(10))

    pos = [5,2]
    robot.setSpeed(0, -30)  # Giro de 45 deg a la derecha
    robot.check_position_3_values(200 + 400 * pos[0], 200 + 400 * pos[1], np.deg2rad(-90), np.Infinity, np.Infinity, np.deg2rad(2))

def s_B_(robot, vel):
    """ La funcion s_B_ realiza la trayectoria de s del mapa B basandose en
        la odometria para detener al robot y comenzar con el siguiente movimiento """  
    
    # Se mueve al centro de la primera celda
    robot.setSpeed(vel, 0)
    robot.check_position_3_values(robot,2200,2600, np.deg2rad(-90),10, 5, np.deg2rad(2))
    r = 400
    v = vel
    w = np.rad2deg((float)(v/r))

    robot.check_position_3_values(200 + 400 * pos[0], 200 + 400 * pos[1], np.deg2rad(-180), np.Infinity, np.Infinity, np.deg2rad(2))

    pos = [5,6]
    robot.setSpeed(0, w)  # Giro de 45 deg a la izquierda
    robot.check_position_3_values(200 + 400 * pos[0], 200 + 400 * pos[1], np.deg2rad(0), np.Infinity, np.Infinity, np.deg2rad(2))

    pos = [5.5,5.5]
    robot.setSpeed(v, 0)  # Tramo recto
    robot.check_position_3_values(200 + 400 * pos[0], 200 + 400 * pos[1], np.deg2rad(180), 10, np.Infinity, np.deg2rad(10))

    pos = [5.5,4.5]
    robot.setSpeed(v, w)  # Primera semicircunferencia
    robot.check_position_3_values(200 + 400 * pos[0], 200 + 400 * pos[1], np.deg2rad(180), 10, np.Infinity, np.deg2rad(10))

    pos = [4.5,3.5]
    robot.setSpeed(v, 0)  # Tramo recto
    robot.check_position_3_values(200 + 400 * pos[0], 200 + 400 * pos[1], np.deg2rad(180), 10, np.Infinity, np.deg2rad(10))
 
    pos = [4.5,2.5]
    robot.setSpeed(v, -w)  # Segunda semicircunferencia
    robot.check_position_3_values(200 + 400 * pos[0], 200 + 400 * pos[1], np.deg2rad(0), 10, 400, np.deg2rad(10))
    
    pos = [5,2]
    robot.setSpeed(v, 0)  # Tramo recto
    robot.check_position_3_values(200 + 400 * pos[0], 200 + 400 * pos[1], np.deg2rad(180), 10, np.Infinity, np.deg2rad(10))
 
    robot.setSpeed(0, -30)  # Giro de 45 deg a la derecha
    robot.check_position_3_values(200 + 400 * pos[0], 200 + 400 * pos[1], np.deg2rad(-90), np.Infinity, np.Infinity, np.deg2rad(2))


def half_semicircle(robot):
    robot.setNewPosition(0, 0, np.deg2rad(90))
    # Comienza el circulo
    r = 400
    v = 100
    w = np.rad2deg((float)(v/r))

    pos = [400,400]
    robot.setSpeed(v, -w)  # Primera semicircunferencia
    robot.check_position_3_values(pos[0], pos[1], np.deg2rad(0), 10, np.Infinity, np.deg2rad(10))

def straight_line(robot):
    robot.setNewPosition(0, 0, np.deg2rad(90))
    v = 100
    pos = [0,400]
    robot.setSpeed(v, 0)  # Primera semicircunferencia
    robot.check_position_3_values(pos[0], pos[1], np.deg2rad(90), np.Infinity, 10, np.deg2rad(90))

def rotate_360(robot):
    robot.setNewPosition(0, 0, np.deg2rad(90))
    robot.setSpeed(0,40)
    time.sleep(0.5)
    robot.check_angle(np.deg2rad(90),np.deg2rad(1))



def main(args):
    try:

        robot = Robot()
        time.sleep(2)
    
        input("Press Enter to continue...")

        # 1. Se incia la odometria u el proceso update odometry
        robot.startOdometry() 

        #half_semicircle(robot)
        straight_line(robot)
        #rotate_360(robot)

        robot.stopOdometry()

    except KeyboardInterrupt:
        # except the program gets interrupted by Ctrl+C on the keyboard.
        # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        robot.stopOdometry()


if __name__ == "__main__":

    # get and parse arguments passed to main
    # Add as many args as you need ...
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--mapa", help="Mapa seleccionado (A o B)", type=str, default="A")
    args = parser.parse_args()

    main(args)

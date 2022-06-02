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

"""
            TRABAJO FINAL:
Autores:
    - Aaron Ibañez Espes 779088
    - Sergio Gabete Cesar 774631
    - Belen Gimeno Chueca 756425
    - Pablo Gancedo Alcalde 736839 
"""
    
def s_A_ocho(robot, vel, err):
    """ La funcion ocho realiza la trayectoria de s del mapa A basandose en
        la odometria para detener al robot y comenzar con el siguiente movimiento """
    robot.write_log()
    # Se mueve al centro de la primera celda
    #robot.go(600,2600, vel)
    robot.setSpeed(vel, 0)  # Giro de 45 deg a la derecha
    robot.check_position_3_values(600,2600, np.deg2rad(-90),np.Infinity, 5, np.deg2rad(10))
    robot.write_log()

    # Comienza el circulo
    r = 400
    v = vel
    w = np.rad2deg((float)(v/r))

    pos = [1,6]
    robot.setSpeed(0, -50)  # Giro de 45 deg a la derecha a 30 (modo normal)
    robot.check_position_3_values(200 + 400 * pos[0], 200 + 400 * pos[1], np.deg2rad(-180), np.Infinity, np.Infinity, np.deg2rad(2))
    robot.write_log()

    pos = [1,4]
    robot.setSpeed(v, w)  # Primera semicircunferencia
    robot.check_position_3_values(200 + 400 * pos[0], 200 + 400 * pos[1], np.deg2rad(0), err, np.Infinity, np.deg2rad(40))
    robot.write_log()

    pos = [1,2]
    robot.setSpeed(v, -w)  # Segunda semicircunferencia
    robot.check_position_3_values(200 + 400 * pos[0], 200 + 400 * pos[1], np.deg2rad(-180), err, 400, np.deg2rad(40))
    robot.write_log()


def s_B_ocho(robot, vel, err):
    """ La funcion ocho realiza la trayectoria de s del mapa B basandose en
        la odometria para detener al robot y comenzar con el siguiente movimiento """
    robot.write_log()
    # Se mueve al centro de la primera celda
    #robot.go(600,2600, vel)
    robot.setSpeed(vel, 0)
    robot.check_position_3_values(2200,2600, np.deg2rad(-90),np.Infinity, 5, np.deg2rad(10))
    robot.write_log()
    # Comienza el circulo
    r = 400
    v = vel
    w = np.rad2deg((float)(v/r))

    pos = [5,6]
    robot.setSpeed(0, 50)  # Giro de 45 deg a la izquierda a 30 en modo normal
    robot.check_position_3_values(200 + 400 * pos[0], 200 + 400 * pos[1], np.deg2rad(0), np.Infinity, np.Infinity, np.deg2rad(2))
    robot.write_log()

    pos = [5,4]
    robot.setSpeed(v, -w)  # Primera semicircunferencia
    robot.check_position_3_values(200 + 400 * pos[0], 200 + 400 * pos[1], np.deg2rad(-180), err, np.Infinity, np.deg2rad(40))
    robot.write_log()

    pos = [5,2]
    robot.setSpeed(v, w)  # Segunda semicircunferencia
    robot.check_position_3_values(200 + 400 * pos[0], 200 + 400 * pos[1], np.deg2rad(0), err, 400, np.deg2rad(40))
    robot.write_log()



def main(args):
    try:
        r2d2 = "R2-D2_s.png"
        bb8 = "BB8_s.png"
        if not os.path.isfile(r2d2):
            print("test image %s does not exist" % r2d2)
            return
        if not os.path.isfile(bb8):
            print("test image %s does not exist" % bb8)
            return

        robot = Robot()
        time.sleep(2)

        mapa = robot.detectar_recorrido()
        
        print("Se ha detectado el mapa ", mapa)
        if mapa is None:
            print('Mapa desconocido, seleccione mapa A o B')
            exit(1)

        print("X value at the beginning from main X= %.2f" % (robot.x.value))   

        myMap = Map2D(mapa)

        if str(args.race_mode) == 'no':
            vel = 120
            err = 10
        else:
            vel = 220
            err = 60

        input("Press Enter to continue...")
        # 1. Se incia la odometria u el proceso update odometry
        robot.startOdometry()

        # 2. perform trajectory
        print("Start : %s" % time.ctime())       

        # Trayectoria en s
        if mapa == "mapaA_CARRERA.txt":  
            s_A_ocho(robot, vel, err)
            solve_relative_map(robot,myMap, [1,2],[3,3], vel)
        
            # # Se inicia la busqueda de la pelota 
            robot.trackObject(colorRangeMin=[0,0,0], colorRangeMax=[255,255,255])
            robot.detect_scape_cv2(vel)
            #Sale despues de coger la pelota
            robot.scape(vel)
            
        else: # "mapaB_CARRERA.txt"
            s_B_ocho(robot, vel, err)
            solve_relative_map(robot,myMap, [5,2],[3,3],vel)
            robot.trackObject(colorRangeMin=[0,0,0], colorRangeMax=[255,255,255])
            robot.detect_scape_cv2(vel) 
            #Sale despues de coger la pelota
            robot.scape(vel)


        print("End : %s" % time.ctime())        


        robot.lock_odometry.acquire()
        print("Odom values at main at the END: %.2f, %.2f, %.2f " %
              (robot.x.value, robot.y.value, robot.th.value))
        robot.lock_odometry.release()
        
        robot.stopOdometry()

    except KeyboardInterrupt:
        # except the program gets interrupted by Ctrl+C on the keyboard.
        # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        robot.stopOdometry()


if __name__ == "__main__":

    # get and parse arguments passed to main
    # Add as many args as you need ...
    parser = argparse.ArgumentParser()
    parser.add_argument("-r", "--race_mode", help="Es carrera", type=str, default="no")
    args = parser.parse_args()

    main(args)

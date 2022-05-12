#!/usr/bin/python
# -*- coding: UTF-8 -*-

import sys
sys.path.append('../libraries')
import os
import cv2
import argparse
from tokenize import Double
import numpy as np
import time
import math
from Robot import Robot
from MapLib import Map2D
from SolveMap import solveMap

"""
            TRABAJO FINAL:
Autores:
    - Aaron Ibañez Espes 779088
    - Sergio Gabete Cesar 774631
    - Belen Gimeno Chueca 756425
    - Pablo Gancedo Alcalde 736839 
"""


def s_A(robot, vel):
    """ La funcion s_A realiza la trayectoria de s del mapa A basandose en
        la odometria para detener al robot y comenzar con el siguiente movimiento """

    pos = [[0,5],[1,4],[2,3],[1,1.8]]
    for point_map in pos:
        point = [200+point_map[0]*400, 200+point_map[1]*400]
        # Se mueve el robot a la siguiente celda
        robot.go(point[0],point[1])
        print('Voy a ',point)

    robot.setSpeed(0, 0)  # Parar el robot

def s_B(robot, vel):
    """ La funcion s_B realiza la trayectoria de s del mapa B basandose en
        la odometria para detener al robot y comenzar con el siguiente movimiento """  
    
    pos = [[5,6],[6,5],[5,4],[4,3],[5,1.5]]
    for point_map in pos:
        point = [200+point_map[0]*400, 200+point_map[1]*400]
        # Se mueve el robot a la siguiente celda
        robot.go(point[0],point[1])
        print('Voy a ',point)


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
        mirror=False

        robot = Robot()

        mapa = robot.detectar_recorrido()
        print("Se ha detectado el mapa ", mapa)
        if mapa is None:
            print('Mapa desconocido, seleccione mapa A o B')
            exit(1)

        print("X value at the beginning from main X= %.2f" % (robot.x.value))   

        myMap = Map2D(mapa)
        input("Press Enter to continue...")
        
        # 1. Se incia la odometria u el proceso update odometry
        robot.startOdometry()
        
        # 2. perform trajectory
        print("Start : %s" % time.ctime())

        # Trayectoria en s
        if mapa == "mapaA_CARRERA.txt":
            imagenFin = cv2.imread(r2d2, cv2.IMREAD_COLOR)
            imagenOtro = cv2.imread(bb8, cv2.IMREAD_COLOR)
            target_robot_file = r2d2
            s_A(robot, 150)
            #Ahora toca corregir la homografia
        else: # "mapaB_CARRERA.txt"
            imagenFin = cv2.imread(bb8, cv2.IMREAD_COLOR)
            imagenOtro = cv2.imread(r2d2, cv2.IMREAD_COLOR)
            target_robot_file = bb8
            s_B(robot, 150)
            #Ahora toca corregir la homografia

        print("End : %s" % time.ctime())

        robot.lock_odometry.acquire()
        print("Odom values at main at the END: %.2f, %.2f, %.2f " %
              (robot.x.value, robot.y.value, robot.th.value))
        robot.lock_odometry.release()

        # 3. Labyrinth

        # 4. Play catch

        # 5. Where is Wall-e?
        # Dejadme hacer magia a mi ritmo FUS FUS
        #match_images(imagenFin, self.capture_image())
        # ÑOFeature extractor uses grayscale images
        # Ñ img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # O cap = cv2.cvtColor(capture, cv2.COLOR_BGR2GRAY)
        #foundFin = robot.detectImage(imagenFin)
        # Gira derecha para buscar el otro robot
        # Si está, girar a izquierda para ir a salida
        # Si no está, es la salida
        #foundOtro = robot.detectImage(imagenOtro)


        # 6. wrap up and close stuff ...
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
    parser.add_argument("-m", "--mapa", help="Mapa seleccionado (A o B)", type=str, default="A")
    args = parser.parse_args()

    main(args)

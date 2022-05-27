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
        time.sleep(2)

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
            # pos = robot.readOdometry()
            # print('Empiezo en', pos)
            # robot.setNewPosition(0,0,np.deg2rad(-180))
            
            #s_A_ocho(robot, 120)
            #solve_relative_map(robot,myMap, [1,2],[3,3])
            
            #solveMap(robot,myMap, [1,2],[3,3])
            robot.setNewPosition(1600,1400,np.deg2rad(90))
            robot.detect_scape()
            # Se inicia la busqueda de la pelota 
            robot.trackObject(colorRangeMin=[0,0,0], colorRangeMax=[255,255,255])

            #Sale desplues de coger la pelota
            #robot.scape()
            
        else: # "mapaB_CARRERA.txt"
            imagenFin = cv2.imread(bb8, cv2.IMREAD_COLOR)
            imagenOtro = cv2.imread(r2d2, cv2.IMREAD_COLOR)
            target_robot_file = bb8
            s_B_ocho(robot, 150)
            solveMap(robot,myMap, [5,2],[3,3])

            #Ahora toca corregir la homografia

        print("End : %s" % time.ctime())

        #Se reorienta el robot
        


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

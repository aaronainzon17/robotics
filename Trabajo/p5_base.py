#!/usr/bin/python
# -*- coding: UTF-8 -*-

import sys
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
from libraries.SolveMap import solveMap
from libraries.BlobDetector import getRedBloobs,detect_red,getGreenBloobs,detect_green
from ficheros_codigo_auxiliar.sample_matching import find_template

"""
            TRABAJO FINAL:
Autores:
    - Aaron Ibañez Espes 779088
    - Sergio Gabete Cesar 774631
    - Belen Gimeno Chueca 756425
    - Pablo Gancedo Alcalde 736839 
"""

def normalizar(th):
    """ Funcion de normalizacion del angulo entre -pi, pi """

    if th > math.pi:
        th -= (2 * math.pi)
    elif th < -math.pi:
        th += (2 * math.pi)
    return th


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





def mov_debug(robot, vel):
    """ La funcion s_A realiza la trayectoria de s del mapa A basandose en
        la odometria para detener al robot y comenzar con el siguiente movimiento """

    pos = [[1,6],[1,5],[1,4]]
    for point_map in pos:
        point = [200+point_map[0]*400, 200+point_map[1]*400]
        # Se mueve el robot a la siguiente celda
        robot.go(point[0],point[1])
        print('Voy a ',point)

    robot.setSpeed(0, 0)  # Parar el robot

def s_A(robot, vel):
    """ La funcion s_A realiza la trayectoria de s del mapa A basandose en
        la odometria para detener al robot y comenzar con el siguiente movimiento """

    pos = [[0,5],[1,4],[2,3],[1,1.8],[1,0]]
    for point_map in pos:
        point = [200+point_map[0]*400, 200+point_map[1]*400]
        # Se mueve el robot a la siguiente celda
        robot.go(point[0],point[1])
        print('Voy a ',point)

    robot.setSpeed(0, 0)  # Parar el robot

def s_B(robot, vel):
    """ La funcion s_B realiza la trayectoria de s del mapa B basandose en
        la odometria para detener al robot y comenzar con el siguiente movimiento """  
    
    pos = [[6,5],[5,4],[4,3],[5,1.8],[5,0]]
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
        time.sleep(1)

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

        #Primero se centra hacia la pared
        #robot.centrar_con_imagen()        

        # Trayectoria en s
        if mapa == "mapaA_CARRERA.txt":
            s_A(robot, 100)
            #[xA,yA,thA]= robot.readOdometry()
            #robot.setSpeed(0, 10)  # cuarto de circunferencia a la derecha
            #check_position(robot, xA, yA, normalizar(np.deg2rad(270)), np.Infinity, np.Infinity, np.deg2rad(2))
            #robot.setSpeed(0,0)
            #[xA,yA,thA]= robot.readOdometry()
            #print("La odometria tras acabar la s es: x= ",xA," y= ",yA," th= ",thA)
            ##Ahora toca corregir la homografia

            #robot.setNewPosition(1400,1200,np.deg2rad(90))
            #robot.scape()
            # Se inicia la busqueda de la pelota 
            #robot.trackObject(colorRangeMin=[0,0,0], colorRangeMax=[255,255,255])
            
        else: # "mapaB_CARRERA.txt"
            imagenFin = cv2.imread(bb8, cv2.IMREAD_COLOR)
            imagenOtro = cv2.imread(r2d2, cv2.IMREAD_COLOR)
            target_robot_file = bb8
            s_B(robot, 150)
            #Ahora toca corregir la homografia

        print("End : %s" % time.ctime())

        #Se reorienta el robot
        #robot.centrar_con_imagen()
        time.sleep(10)


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

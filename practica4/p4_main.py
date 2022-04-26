#!/usr/bin/python
# -*- coding: UTF-8 -*-
from MapLib import Map2D
from Robot import Robot
import argparse
import os
import numpy as np
import time

import matplotlib
matplotlib.use("TkAgg")
# sudo apt-get install tcl-dev tk-dev python-tk python3-tk if TkAgg is not available

# from Robot import Robot

# NOTES ABOUT TASKS to DO in P4:
# 1)findPath(x1,y1, x2,y2),   fillCostMatrix(), replanPath () --> should be methods from the new Map2D class
# 2) go(x,y) and detectObstacle() could be part of your Robot class (depending how you have implemented things)
# 3) you can change these method signatures if you need, depending how you have implemented things

def solveMap(robot, map_file, point_ini, point_end, ocho=False):
    # 1. load map and compute costs and path
    myMap = Map2D(map_file)
    myMap.fillCostMatrix(3, 3, True)

def main(args):
    
    try:
        if not os.path.isfile(args.mapfile):
            print('Map file %s does not exist' % args.mapfile)
            exit(1)

        map_file = args.mapfile
        
        # Punto inicial, objetivo y 8-vecindad
        ini = [0,0]
        goal = [4,2]     
        ocho = True     

        # Se calcula la posicion en la realidad del robot para la coordenada inicial
        ini_point = [200+point_map[0]*400, 200+point_map[1]*400, 1.57]
        # Se instancia el robot
        robot = Robot(init_position=[ini_point[0],ini_point[1],np.deg2rad(90)]) 
        
        # Se inicia la odometria
        robot.startOdometry()

        # 1. load map and compute costs and path
        myMap = Map2D(map_file)
        
        # Se calcula el camino inicial
        if not myMap.findPath(ini,goal, ocho):
            # En caso de que no exista en camino a la celda indicada
            print('NO EXISTE CAMINO DISPONIBLE')
            robot.setSpeed(0,45)
            time.sleep(5)
            robot.setSpeed(0,0)
            exit(1)
        
        path2print = []
        goal_reached = False
        lost = False
        # Se intenta resolver el camino hasta que se consiga o no haya camino disponible 
        # por los obstaculos no registrados 
        while not goal_reached and not lost:

            prev_point = np.array(ini)
            for point_map in myMap.currentPath:
                point = [200+point_map[0]*400, 200+point_map[1]*400, 1.57]
                # Si de detecta obstaculo 
                if robot.detectObstacle(point[0],point[1]):
                    
                    robot.setSpeed(0,0)
                    [_,_,th] = robot.readOdometry()
                    # se anyade el obsatculo al mapa 
                    ocho = myMap.setNewObstacle(prev_point, th)
                    
                    # Se recalcula el camino con el nuevo obstaculo 
                    if not myMap.replanPath(prev_point[0],prev_point[1],goal[0], goal[1],ocho):
                        # En caso de que no exista en camino a la celda indicada
                        print('NO EXISTE CAMINO DISPONIBLE CON EL NUEVO OBSTACULO DETECTADO')
                        robot.setSpeed(0,45)
                        time.sleep(5)
                        robot.setSpeed(0,0)
                        # Se indica que el robot se ha "perdido"
                        lost = True
                    
                    goal_reached = False
                    break
                else:
                    goal_reached = True

                # Se mueve el robot a la siguiente celda
                robot.go(point[0],point[1])
                path2print.append([point[0], point[1], 1.57])
                prev_point = point_map
        
        # Al acabar se muestra el recorrido realizado
        myMap.drawMapWithRobotLocations(
            path2print, saveSnapshot=False)
        
        # Se detiene la odometria
        robot.stopOdometry()

    except KeyboardInterrupt: 
    # except the program gets interrupted by Ctrl+C on the keyboard.
    # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        robot.stopOdometry()


if __name__ == "__main__":

    # get and parse arguments passed to main
    # Add as many args as you need ...
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--mapfile", help="path to find map file",
                        default="mapa2.txt")
    args = parser.parse_args()
    main(args)
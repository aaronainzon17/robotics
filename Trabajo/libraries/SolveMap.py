#!/usr/bin/python
# -*- coding: UTF-8 -*-
from libraries.MapLib import Map2D
import numpy as np
import time

import matplotlib
#matplotlib.use("TkAgg")

def solveMap(robot, myMap, point_ini, point_end, ocho=False):
    # 1. load map and compute costs and path
    #myMap = Map2D(map_file)
    
    # Se calcula el camino inicial
    if not myMap.findPath(point_ini,point_end, ocho):
        # En caso de que no exista en camino a la celda indicada
        print('NO EXISTE CAMINO DISPONIBLE')
        robot.setSpeed(0,45)
        time.sleep(5)
        robot.setSpeed(0,0)
        exit(1)
    print("camino calculado")
    path2print = []
    goal_reached = False
    lost = False
    # Se intenta resolver el camino hasta que se consiga o no haya camino disponible 
    # por los obstaculos no registrados 
    while not goal_reached and not lost:

        prev_point = np.array(point_ini)
        for point_map in myMap.currentPath:
            point = [200+point_map[0]*400, 200+point_map[1]*400, 1.57]
            
            # Si de detecta obstaculo 
            if robot.detectObstacle(point[0],point[1]):
                robot.setSpeed(0,0)
                [_,_,th] = robot.readOdometry()
                # se anyade el obsatculo al mapa 
                ocho = myMap.setNewObstacle(prev_point, th, ocho)
                
                # Se recalcula el camino con el nuevo obstaculo 
                if not myMap.replanPath(prev_point[0],prev_point[1],point_end[0], point_end[1],ocho):
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
            print('Voy al go')
            # Se mueve el robot a la siguiente celda
            robot.go(point[0],point[1],100)
            path2print.append([point[0], point[1], 1.57])
            prev_point = point_map
    
    # Al acabar se muestra el recorrido realizado
    #myMap.drawMapWithRobotLocations(
    #    path2print, saveSnapshot=False)

def solve_relative_map(robot, myMap, point_ini, point_end, vel, ocho=False):
    # 1. load map and compute costs and path
    #myMap = Map2D(map_file)
    
    # Se calcula el camino inicial
    if not myMap.findPath(point_ini,point_end, ocho):
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
    prev_point = np.array(point_ini)
    while not goal_reached and not lost:

        for point_map in myMap.currentPath:
            point = np.array([200+point_map[0]*400, 200+point_map[1]*400])
            point_prev = np.array([200+prev_point[0]*400, 200+prev_point[1]*400])
            point_now = np.array(robot.readOdometry())
            goal_point = [point_now[0],point_now[1]] + (point - point_prev)
            
            # Si de detecta obstaculo 
            if robot.detectObstacle(goal_point[0],goal_point[1]):
                robot.setSpeed(0,0)
                [_,_,th] = robot.readOdometry()
                # se anyade el obsatculo al mapa 
                ocho = myMap.setNewObstacle(prev_point, th, ocho)
                print('coloco obstaculo en', prev_point)
                # Se recalcula el camino con el nuevo obstaculo 
                if not myMap.replanPath(prev_point[0],prev_point[1],point_end[0], point_end[1],ocho):
                    # En caso de que no exista en camino a la celda indicada
                    print('NO EXISTE CAMINO DISPONIBLE CON EL NUEVO OBSTACULO DETECTADO')
                    robot.setSpeed(0,45)
                    time.sleep(5)
                    robot.setSpeed(0,0)
                    # Se indica que el robot se ha "perdido"
                    lost = True
                
                goal_reached = False
                #prev_point = point_map
                break
            else:
                goal_reached = True

            # Se mueve el robot a la siguiente celda
            robot.go(goal_point[0],goal_point[1],vel)
            #path2print.append([goal_point[0],goal_point[1], 1.57])
            prev_point = point_map
    
    # Al acabar se muestra el recorrido realizado
    #myMap.drawMapWithRobotLocations(
    #    path2print, saveSnapshot=False)
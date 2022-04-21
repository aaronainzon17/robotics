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
        
        # Se instancia el robot
        robot = Robot(init_position=[200,0,np.deg2rad(90)]) 
        # Se inicia la odometria
        robot.startOdometry()

        # 1. load map and compute costs and path
        myMap = Map2D(map_file)
        myMap.fillCostMatrix(2, 2, True)
        #myMap.verbose = True
        #myMap.drawMap(saveSnapshot=False)
        myMap.findPath([0,0],[2,2], True)
        
        print(myMap.currentPath)
        path2print = []
        for i in myMap.currentPath:
            if i[0] == 0 and i[1] == 0: 
                path2print.append([200, 200, 1.57])
            elif i[0] == 0: 
                path2print.append([200, 200+i[1]*400, 1.57])
            elif i[1] == 0:
                path2print.append([200+i[0]*400, 200,1.57]) 
            else:
                path2print.append([200+i[0]*400, 200+i[1]*400, 1.57])
        prev_point = np.array([200,0])
        for point in path2print:
            point_now = np.array(robot.readOdometry())
            #point = np.array(point[:2])
            #goal_point = point_now[:2] + (point - prev_point)
            #print('Voy al punto', goal_point, 'desde', point_now)
            robot.go(point[0],point[1])
            prev_point = point

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
                        default="mapa0.txt")
    args = parser.parse_args()
    main(args)
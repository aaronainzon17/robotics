#!/usr/bin/python
# -*- coding: UTF-8 -*-
from MapLib import Map2D
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


def main(args):
    """
    Example to load "mapa1.txt"
    """

    try:
        if not os.path.isfile(args.mapfile):
            print('Map file %s does not exist' % args.mapfile)
            exit(1)

        map_file = args.mapfile
        # Instantiate Odometry with your own files from P2/P3
        # robot = Robot()
        # ...
        ini = [0,0]
        goal = [6,4]   
        ocho = True  
        # 1. load map and compute costs and path
        myMap = Map2D(map_file)
        
        #myMap.verbose = True
        #myMap.drawMap(saveSnapshot=False)
        
        myMap.findPath(ini,goal, ocho)
        
        myMap.setNewObstacleCenter([0,3],np.deg2rad(0), ocho)
        
        myMap.replanPath(ini[0], ini[1], goal[0], goal[1], ocho)
        
        myMap.drawMap(saveSnapshot=False)
        
        #print(myMap.currentPath)
        path2print = []
        for i in myMap.currentPath:
        #    if i == None:
        #        print('No veo camino')
        #        break
        #    else:
            path2print.append([200+i[0]*400, 200+i[1]*400, np.deg2rad(-130)])
        #    #if i[0] == 0 and i[1] == 0: 
        #    #    path2print.append([200, 200, 1.57])
        #    #elif i[0] == 0: 
        #    #    path2print.append([200, 200+i[1]*400, 1.57])
        #    #elif i[1] == 0:
        #    #    path2print.append([200+i[0]*400, 200,1.57]) 
        #    #else:
        #    #    path2print.append([200+i[0]*400, 200+i[1]*400, 1.57])
        #
        myMap.drawMapWithRobotLocations(path2print, saveSnapshot=False)
        
        return

        # you can set verbose to False to stop displaying plots interactively
        # (and maybe just save the snapshots of the map)
        # myMap.verbose = False

        # sample commands to see how to draw the map
        sampleRobotLocations = [[0, 0, 0], [600, 600, 3.14]]
        # this will save a .png with the current map visualization,
        #??all robot positions, last one in green
        #myMap.verbose = True
        myMap.drawMapWithRobotLocations(
            sampleRobotLocations, saveSnapshot=False)

        # this shows the current, and empty, map and an additionally closed connection
        myMap.deleteConnection(0, 0, 0)
        #myMap.verbose = True
        myMap.drawMap(saveSnapshot=False)

        # this will open a window with the results, but does not work well remotely
        #myMap.verbose = True
        sampleRobotLocations = [[200, 200, 3.14/2.0],
                                [200, 600, 3.14/4.0], [200, 1000, -3.14/2.0], ]
        myMap.drawMapWithRobotLocations(
            sampleRobotLocations, saveSnapshot=False)

        matplotlib.pyplot.close('all')
        # 2. launch updateOdometry thread()
        # robot.startOdometry()
        # ...

        # 3. perform trajectory
        # robot.setSpeed(1,1) ...
        # while (notfinished){

        # robot.go(pathX[i],pathY[i]);
        # check if there are close obstacles
        # deal with them...
        # Avoid_obstacle(...) OR RePlanPath(...)

        # 4. wrap up and close stuff ...
        # This currently unconfigure the sensors, disable the motors,
        # and restore the LED to the control of the BrickPi3 firmware.
        # robot.stopOdometry()

    except KeyboardInterrupt:
        # except the program gets interrupted by Ctrl+C on the keyboard.
        # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        #    robot.stopOdometry()
        print('do something to stop Robot when we Ctrl+C ...')


if __name__ == "__main__":

    # get and parse arguments passed to main
    # Add as many args as you need ...
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--mapfile", help="path to find map file",
                        default="mapa2.txt")
    args = parser.parse_args()
    main(args)

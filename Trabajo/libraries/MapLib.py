#!/usr/bin/python
# -*- coding: UTF-8 -*-
# use python 3 syntax but make it compatible with python 2
from __future__ import print_function
from __future__ import division
from tkinter import Y
import wave  # ''

import matplotlib.pyplot as plt
from matplotlib import animation
import numpy as np
import time
import os
import queue


class Map2D:
    def __init__(self, map_description_file):
        """
        Load and initialize map from file. \

        map_description_file: path to a text file containing map description in the standard format. \
        Example for a 3x3 grid map, with (squared) cells of 400mm side length called mapa0. \
        All free space, i.e., all connections between cells are open, except those on the limits of the map.
        For more details on the format, see class documentation.

        mapa0.txt content:
        3 3 400
        0 0 0 0 0 0 0
        0 1 1 1 1 1 0
        0 1 1 1 1 1 0
        0 1 1 1 1 1 0
        0 1 1 1 1 1 0
        0 1 1 1 1 1 0
        0 0 0 0 0 0 0

        """
        # params to visualize
        self.mapLineStyle = 'r-'
        self.costValueStyle = 'g*'
        self.verbose = True
        # set to False to stop displaying plots interactively (and maybe just save the screenshots)
        # self.verbose = False
        self.current_ax = None

        # variables about map params
        self.sizeX = 0
        self.sizeY = 0
        self.sizeCell = 0

        self.connectionMatrix = None
        self.costMatrix = None
        self.currentPath = None

        if self._loadMap(map_description_file):
            print("Map %s loaded ok" % map_description_file)
        else:
            print("Map %s NOT loaded" % map_description_file)

    # from python docs: https://docs.python.org/3/tutorial/classes.html#private-variables
    # “Private” instance variables that cannot be accessed except from inside an object don’t exist in Python.
    # However, there is a convention that is followed by most Python code: a name prefixed with an underscore \
    # (e.g. _spam) should be treated as a non-public part of the API (whether it is a function, a method or a data member).

    # ############################################################
    # private methods
    # ############################################################

    def _initConnections(self, init_value=0):
        """
        to initialize the matrix, we set all connections to be closed.
        When the file with the description is loaded, it will "open" (set to 1) the corresponding ones.
        """
        self.connectionMatrix = np.ones(
            (2*self.sizeX+1, 2*self.sizeY+1)) * init_value

    def _initCostMatrix(self, init_value=-2):
        """
        to initialize the matrix, we set all connections to be closed.
        When the file with the description is loaded, it will "open" (set to 1) the corresponding ones.
        """
        self.costMatrix = np.ones((self.sizeX, self.sizeY)) * init_value

        # Example costMatrix (filled manually!) for Map1
        # if we plan to go from 0,0 to 2,0
        # self.costMatrix[2,0] = 0
        # self.costMatrix[1,0] = 1
        # self.costMatrix[1,1] = 2
        # self.costMatrix[1,2] = 3
        # self.costMatrix[0,2] = 4
        # self.costMatrix[2,2] = 4
        # self.costMatrix[0,1] = 5
        # self.costMatrix[2,1] = 5
        # self.costMatrix[0,0] = 6

    def _loadMap(self, mapFileName):
        """
        Load map from a txt file (mapFileName) to fill the map params and connectionMatrix. \
        NOTES: \
        \t connectionMatrix is a numpy array \
        \t Function will return False if something went wrong loading the map file.
        """
        try:
            # FILL GLOBAL VARIABLES dimX dimY cellSize
            loadingOk = False
            mapF = open(mapFileName, "r")

            # 1. special case for first line. initialize dimX dimY cellSize
            header = mapF.readline()  # next()
            # any whitespace string is a separator and empty strings are removed from the result
            tmp = header.split()
            if self.verbose:
                print("Header line: %s " % header)
            parsed_header = [int(c) for c in tmp]
            # expected to have three numbers: sizeX sizeY sizeCell_in_mm
            if len(parsed_header) == 3:
                self.sizeX, self.sizeY, self.sizeCell = parsed_header
            else:
                print("Wrong header in map file: %s" % header)
                return False

            # 2.init connectionMatrix and costMatrix
            self._initConnections()
            self._initCostMatrix()

            # 3. load rest of the map connection lines information
            for indx, line in enumerate(mapF):
                # we start loading from the file the "top" row of the map
                current_row = (self.connectionMatrix.shape[1]-1) - indx
                # Split numbers in the line. Any whitespace string is a separator and empty strings are removed from the result
                tmp = line.split()
                if self.verbose:
                    print("Line for map row %d: %s " % (current_row, line))
                parsed_line = [int(c) for c in tmp]

                if len(parsed_line) == self.connectionMatrix.shape[0] and indx < self.connectionMatrix.shape[1]:
                    self.connectionMatrix[:, current_row] = parsed_line
                elif len(parsed_line):  # don't give errors because of empty lines
                    print("Wrong connectionMatrix (%s) row data: %s" %
                          (self.connectionMatrix.shape(), line))
                    return False
            mapF.close()
            loadingOk = True
        except Exception as e:
            print("ERROR:", e.__doc__)
            print(e)
            # raise
            loadingOk = False

        return loadingOk

    def _cell2connCoord(self, cellX, cellY, numNeigh):
        """
        Input:
            cellX, cellY: cell coordinates (cellX, cellY) in the map grid
            numNeigh: index of one of the cell 8-neighbours

        Output:
            (connX,connY): 2D coordinates (in the connectionMatrix!!) \
            of the connection of the input cell to the input neighbour
        """
        connX = 2*cellX+1
        connY = 2*cellY+1
        p = [connX, connY]

        result = {
            0: lambda p: [p[0],    p[1]+1],
            1: lambda p: [p[0]+1,  p[1]+1],
            2: lambda p: [p[0]+1,  p[1]],
            3: lambda p: [p[0]+1,  p[1]-1],
            4: lambda p: [p[0],    p[1]-1],
            5: lambda p: [p[0]-1,  p[1]-1],
            6: lambda p: [p[0]-1,  p[1]],
            7: lambda p: [p[0]-1,  p[1]+1],
        }

        return result[numNeigh](p)

    def _neighbour(self, cellX, cellY, numNeigh):
        """
        Input:
            cellX, cellY: cell coordinates (cellX, cellY) in the map grid
            numNeigh: index of one of the cell 8-neighbours

        Output:
            (connX,connY): 2D coordinates (in the connectionMatrix!!) \
            of the connection of the input cell to the input neighbour
        """
        #connX = 2*cellX+1
        #connY = 2*cellY+1
        p = [cellX, cellY]

        result = {
            0: lambda p: [p[0],    p[1]+1],
            1: lambda p: [p[0]+1,  p[1]+1],
            2: lambda p: [p[0]+1,  p[1]],
            3: lambda p: [p[0]+1,  p[1]-1],
            4: lambda p: [p[0],    p[1]-1],
            5: lambda p: [p[0]-1,  p[1]-1],
            6: lambda p: [p[0]-1,  p[1]],
            7: lambda p: [p[0]-1,  p[1]+1],
        }

        return result[numNeigh](p)

    def _pos2cell(self, x_mm, y_mm):
        """ Convert from robot odometry coordinates (in mm) to cell coordinates """
        # make sure we discretize the result to the closest lower integer value
        x_cell = int(np.floor(x_mm/self.sizeCell))
        y_cell = int(np.floor(y_mm/self.sizeCell))
        return [x_cell, y_cell]

    # ############################################################
    # public methods
    # ############################################################

    def setConnection(self, cellX, cellY, numNeigh):
        """
        open a connection, i.e., we can go straight from cellX,cellY to its neighbour number numNeigh
        """
        # from coordinates in the grid of cells to coordinates in the connection matrix
        [connX, connY] = self._cell2connCoord(cellX, cellY, numNeigh)
        self.connectionMatrix[connX, connY] = 1  # True

    def deleteConnection(self, cellX, cellY, numNeigh):
        """
        close a connection, i.e., we can NOT go straight from cellX,cellY to its neighbour number numNeigh
        """
        # from coordinates in the grid of cells to coordinates in the connection matrix
        [connX, connY] = self._cell2connCoord(cellX, cellY, numNeigh)
        self.connectionMatrix[connX, connY] = 0  # False

    def isConnected(self, cellX, cellY, numNeigh):
        """
        returns True if the connnection from cell (x,y) to its neighbour number numNeigh is open.

        The neighbour indexing is considered as follows
        (8-neighbours from cell x,y numbered clock-wise):

        7     0       1
        6   (x,y)     2
        5     4       3

        """
        [connX, connY] = self._cell2connCoord(cellX, cellY, numNeigh)
        return self.connectionMatrix[connX, connY]

    # aux functions to display (or save image) with robot and map stuff
    def _drawGrid(self):
        """
        aux function to create a grid with map lines
        """
        if not self.current_ax:
            print("Error plotting: do not call this function directly, \
                call drawMap first to create a plot where to draw")
            return False

        plt.rc('grid', linestyle="--", color='gray')
        plt.grid(True)
        plt.tight_layout()

        x_t = range(0, (self.sizeX+1)*400, 400)
        y_t = range(0, (self.sizeY+1)*400, 400)
        x_labels = [str(n) for n in x_t]
        y_labels = [str(n) for n in y_t]
        plt.xticks(x_t, x_labels)
        plt.yticks(y_t, y_labels)

        # Main rectangle
        X = np.array([0, self.sizeX, self.sizeX,
                      0,          0]) * self.sizeCell
        Y = np.array([0, 0,          self.sizeY,
                      self.sizeY, 0]) * self.sizeCell
        self.current_ax.plot(X, Y, self.mapLineStyle)

        # "vertical" walls
        for i in range(2, 2*self.sizeX, 2):
            for j in range(1, 2*self.sizeY, 2):
                if not self.connectionMatrix[i, j]:
                    # paint "right" wall from cell (i-1)/2, (j-1)/2
                    cx = np.floor((i-1)/2)
                    cy = np.floor((j-1)/2)
                    X = np.array([cx+1, cx+1]) * self.sizeCell
                    Y = np.array([cy, cy+1]) * self.sizeCell
                    self.current_ax.plot(X, Y, self.mapLineStyle)

        # "horizontal" walls
        for j in range(2, 2*self.sizeY, 2):
            for i in range(1, 2*self.sizeX, 2):
                if not self.connectionMatrix[i, j]:
                    # paint "top" wall from cell (i-1)/2, (j-1)/2
                    cx = np.floor((i-1)/2)
                    cy = np.floor((j-1)/2)
                    X = np.array([cx, cx+1]) * self.sizeCell
                    Y = np.array([cy+1, cy+1]) * self.sizeCell
                    self.current_ax.plot(X, Y, self.mapLineStyle)
        plt.axis('equal')

        return True

    # aux functions to display the current CostMatrix on the map

    def _drawCostMatrix(self):
        """
        aux function to create a grid with map lines
        """
        if not self.current_ax:
            print("Error plotting: do not call this function directly, \
                call drawMap first to create a plot where to draw")
            return False

        # "center" of each cell
        for i in range(0, self.sizeX):
            for j in range(0, self.sizeY):
                cx = i*self.sizeCell + self.sizeCell/2.
                cy = j*self.sizeCell + self.sizeCell/2.
                X = np.array([cx])
                Y = np.array([cy])
                cost = self.costMatrix[i, j]
                self.current_ax.text(X, Y, str(cost))

        plt.axis('equal')

        return True

    # Dibuja robot en location_eje con color (c) y tamano (p/g)
    def _drawRobot(self, loc_x_y_th=[0, 0, 0], robotPlotStyle='b', small=False):
        """
        UPDATES existing plot to include current robot position
        It expects an existing open figure (probably with the map already on it)

        loc_x_y_th is the position x,y and orientation in mm and radians of the main axis of the robot

        """
        if not self.current_ax:
            print("Error plotting: do not call this function directly, \
                call drawMap first to create a plot where to draw")
            return False

        if small:
            largo, corto, descentre = [80, 50, 5]
        else:
            largo, corto, descentre = [160, 100, 10]

        trasera_dcha = np.array([-largo, -corto, 1])
        trasera_izda = np.array([-largo, corto, 1])
        delantera_dcha = np.array([largo, -corto, 1])
        delantera_izda = np.array([largo, corto, 1])
        frontal_robot = np.array([largo, 0, 1])

        tita = loc_x_y_th[2]
        Hwe = np.array([[np.cos(tita), -np.sin(tita), loc_x_y_th[0]],
                        [np.sin(tita), np.cos(tita), loc_x_y_th[1]],
                        [0,        0,        1]])

        Hec = np.array([[1, 0, descentre],
                        [0, 1, 0],
                        [0, 0, 1]])

        extremos = np.array([trasera_izda, delantera_izda, delantera_dcha,
                             trasera_dcha, trasera_izda, frontal_robot, trasera_dcha])
        robot = np.dot(Hwe, np.dot(Hec, np.transpose(extremos)))

        self.current_ax.plot(robot[0, :], robot[1, :], robotPlotStyle)

        return True

    def drawMapWithRobotLocations(self,
                                  robotPosVectors=[
                                      [0, 0, 0], [600, 600, 3.14]],
                                  saveSnapshot=True):
        """ Overloaded version of drawMap to include robot positions """
        return self.drawMap(robotPosVectors=robotPosVectors, saveSnapshot=saveSnapshot)

    def drawMap(self, robotPosVectors=None, saveSnapshot=False):
        """
        Generates a plot with currently loaded map status

        NOTE:
        if verbose, it displays the plot
        if saveSnapshot: saves a figure as mapstatus_currenttimestamp_FIGNUM.png
        """
        self.verbose = True
        # self.verbose=False

        # create a new figure and set it as current axis
        current_fig = plt.figure()
        self.current_ax = current_fig.add_subplot(111)

        self._drawGrid()

        # if flag is true, draw also current CostMatrix
        if self.verbose:
            self._drawCostMatrix()

        if robotPosVectors:
            for loc in robotPosVectors:
                print("Robot in pos: ", loc)
                self._drawRobot(loc_x_y_th=loc, robotPlotStyle='b--')
            # plot last robot position with solid green line
            self._drawRobot(loc_x_y_th=loc, robotPlotStyle='g-')

        if saveSnapshot:
            ts = str(time.time())
            snapshot_name = "mapstatus_"+ts+"_F"+str(current_fig.number)+".png"
            print("saving %s " % snapshot_name)
            plt.savefig(snapshot_name)

        if self.verbose:
            current_fig.set_visible(True)
            current_fig.show()
            print("Press ENTER in the plot window to continue ... ")
            current_fig.waitforbuttonpress()
        else:
            current_fig.set_visible(False)

        return current_fig

    def findPath(self, point_ini, point_end, ocho=False):
        """ overloaded call to planPath (x_ini,  y_ini, x_end, y_end) """
        return self.planPath(point_ini[0], point_ini[1],
                             point_end[0], point_end[1], ocho)

    # ############################################################
    # METHODS to IMPLEMENT in P4
    # ############################################################

    # Rellena la matriz de coste del mapa
    def fillCostMatrix(self, x_end, y_end, ocho=False):
        # Se crea una cola en la que se guardan las celdas las cuales 
        # sus vecinas se tienen que rellenar
        wavefront = queue.Queue()
        wavefront.put([x_end, y_end])
        self.costMatrix[x_end][y_end] = 0
        
        # Mientras queden celdas por rellenar 
        while not wavefront.empty():
            [x, y] = wavefront.get()
            val = self.costMatrix[x][y]

            self._checkCell(x, y+1, self._cell2connCoord(x, y, 0), wavefront, val)
            self._checkCell(x+1, y, self._cell2connCoord(x, y, 2), wavefront, val)
            self._checkCell(x, y-1, self._cell2connCoord(x, y, 4), wavefront, val)
            self._checkCell(x-1, y, self._cell2connCoord(x, y, 6), wavefront, val)
            
            # Si se quiere realizar 8 vecindad
            if ocho:
                self._checkCell(x+1, y+1, self._cell2connCoord(x, y, 1), wavefront, val)
                self._checkCell(x+1, y-1, self._cell2connCoord(x, y, 3), wavefront, val)
                self._checkCell(x-1, y-1, self._cell2connCoord(x, y, 5), wavefront, val)
                self._checkCell(x-1, y+1, self._cell2connCoord(x, y, 7), wavefront, val)

    # En esta funcion se comprueba si ya se le ha asignado previamente peso a la celda
    # en caso de que no, se le asigna un peso y se anyade a la cola para hacer lo mismo con 
    # sus vecinas
    def _checkCell(self, x, y, conn, wavefront, val):
        if x >= 0 and x <= (self.sizeX-1) and y >= 0 and y <= (self.sizeY-1):
            # Si todavia no se le ha asignado un peso y no es obstaculo
            if self.costMatrix[x][y] == -2 and self.connectionMatrix[conn[0]][conn[1]] == 1:
                # Se anyade a la cola 
                wavefront.put([x, y])
                # Se le da un peso de val + 1 
                self.costMatrix[x][y] = val + 1
                
    # Calcula el recorrido basandose en los pesos calculados en la matriz de costes 
    def planPath(self, x_ini,  y_ini, x_end, y_end, ocho=False):
        """
        x_ini, y_ini, x_end, y_end: integer values that indicate \
        the x and y coordinates of the starting (ini) and ending (end) cell
        """
        # Se inicializa el nuevo recorrido con la coordenada de partida 
        #REVISAR
        self.currentPath = [[x_ini, y_ini]] 
        pathFound = False 
        existePath = True
        # Se rellena la matriz usando NF1
        self.fillCostMatrix(x_end, y_end, ocho)
        # Se recalcula el camino
        while not pathFound and existePath:
            if self.currentPath[len(self.currentPath) - 1] is not None:
                # Si existe el camino
                [x,y] = self.currentPath[len(self.currentPath) - 1]
                # Se anyade al path el vecino de menor peso
                nextStep = self.min_neighbour(x,y,self.costMatrix[x][y],ocho)
                self.currentPath.append(nextStep)
                if [x_end,y_end] == nextStep:
                    pathFound = True
            else: 
                # Si no existe camino posibe 
                existePath = False
        #REVISAR
        self.currentPath = self.currentPath#[1:]
        if existePath:
            return pathFound
        else:
            return False

    # Devuelve la coordenada de la celda vecina de menor coste
    def min_neighbour(self, x, y, min, ocho=False):
        if ocho:
            values = list(range(0,7))
        else:
            values = list(range(0,7,2))

        minCoord = None
        for i in values:
            cm = self._cell2connCoord(x, y, i) # cm = connection matrix 
            if x >= 0 and x <= (self.sizeX-1) and y >= 0 and y <= (self.sizeY-1) and self.connectionMatrix[cm[0]][cm[1]] == 1: # si la celda del vecino i no es obstaculo
                [x_n,y_n] = self._neighbour(x,y,i) # Se obtienen las coordenadas de la celda en la matriz de costes
                # se compara con la minima
                if self.costMatrix[x_n][y_n] < min:
                    minCoord = [x_n, y_n]
                    min = self.costMatrix[x_n][y_n]
        
        return minCoord
    
    # Replanifica el recorrido tras encontrar un obstaculo no conocido
    def replanPath(self, x_ini,  y_ini, x_end, y_end, ocho=False):
        self._initCostMatrix()
        return self.planPath(x_ini, y_ini, x_end, y_end, ocho)

    # Anyade un nuevo obstaculo a la matriz de conexiones del mapa
    def setNewObstacleWall(self, point, th, ocho):
        neigbours = self.get_neighbours(th)
        if neigbours != None:
            # Se introduce el obstaculo   
            for cell in neigbours:
                cm = self._cell2connCoord(point[0], point[1], cell) # cm = connection matrix
                self.connectionMatrix[cm[0]][cm[1]] = 0
            if ocho:
                return True
            else:
                return False
        else:
            # Si no es claro donde esta el obstaculo, no se introduce y se planea 4 vecindad
            return False

    # Anyade un nuevo obstaculo a la matriz de conexiones del mapa en el centro de la siguiente celda
    def setNewObstacleCenter(self, point, th, ocho):
        neigbours = list(range(8))
        
        # Se introduce el obstaculo   
        for cell in neigbours:
            cm = self._cell2connCoord(point[0], point[1], cell) # cm = connection matrix
            self.connectionMatrix[cm[0]][cm[1]] = 0
        if ocho:
            return True
        else:
            return False


    # Devuelve en que vecinos de la celda se encuentra el obstaculo, 
    # en caso indeterminado devuelve None
    def get_neighbours(self,th):
        error = np.deg2rad(15)
        
        if abs(th - np.deg2rad(90)) < error:
            # Esta mirando al frente (el obstaculo esta en los vecinos 7,0,1)
            return [7,0,1]
        elif abs(th + np.deg2rad(90)) < error:
            # Esta mirando hacia abajo (el obstaculo esta en los vecinos 3,4,5)
            return [3,4,5]
        elif abs(th) < error: 
            # Esta mirando a la derecha (el obstaculo esta en los vecinos 1,2,3)
            return [1,2,3]
        elif abs(th - np.deg2rad(180)) < error:
            # Esta mirando a la izquierda (el obstaculo esta en los vecinos 5,6,7)
            return [5,6,7]
        elif abs(th + np.deg2rad(180)) < error:
            # Esta mirando a la izquierda (el obstaculo esta en los vecinos 5,6,7)
            return [5,6,7]
        else:
            # En caso de que el obstaculo no este ni al frente ni a la derecha 
            # Se asume que no se sabe donde esta y se acaba el camino haciendo 4 vecindad 
            # para evitar ver obstaculos en diagonal
            return None

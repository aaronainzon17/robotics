#!/usr/bin/python
# -*- coding: UTF-8 -*-
# use python 3 syntax but make it compatible with python 2
from __future__ import print_function
from __future__ import division

# import brickpi3 # import the BrickPi3 drivers
import time     # import the time library for the sleep function
import brickpi3  # import the BrickPi3 drivers
import sys
import numpy as np
import math
import os
import cv2

# tambien se podria utilizar el paquete de threading
from multiprocessing import Process, Value, Array, Lock

from get_color_blobs import getRedBloobs


class Robot:
    def __init__(self, init_position=[0.0, 0.0, 0.0]):
        """
        Initialize basic robot params. \

        Initialize Motors and Sensors according to the set up in your robot
        """

######## UNCOMMENT and FILL UP all you think is necessary (following the suggested scheme) ########

        # Robot construction parameters

        # Radio de la rueda y distancia entre ruedas
        self.R = 28
        self.L = 128

        # Ultimo valor leido de cada motor
        self.acum_d = 0
        self.acum_i = 0

        # Velocidades
        self.v = Value('d', 0.0)
        self.w = Value('d', 0.0)

        ##################################################
        # Motors and sensors setup

        # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
        self.BP = brickpi3.BrickPi3()

        # Configure sensors, for example a touch sensor.
        #self.BP.set_sensor_type(self.BP.PORT_1, self.BP.SENSOR_TYPE.TOUCH)

        # reset encoder B and C (or all the motors you are using)
        self.BP.offset_motor_encoder(self.BP.PORT_B,
                                     self.BP.get_motor_encoder(self.BP.PORT_B))  # RUEDA DERECHA
        self.BP.offset_motor_encoder(self.BP.PORT_C,
                                     self.BP.get_motor_encoder(self.BP.PORT_C))  # RUEDA IZQUIERDA

        ##################################################

        # odometry shared memory values
        self.x = Value('d', init_position[0])
        self.y = Value('d', init_position[1])
        self.th = Value('d', init_position[2])
        # boolean to show if odometry updates are finished
        self.finished = Value('b', 1)

        # if we want to block several instructions to be run together, we may want to use an explicit Lock
        self.lock_odometry = Lock()

        # odometry update period
        self.P = 0.03

        # Variables de vision 
        

    def setSpeed(self, v, w):
        """ Funcion que establece la velocidad lineal del robot a v y la velocidad
            angular del robot a w """

        print("setting speed to %.2f %.2f" % (v, w))

        # Calculo de la velocidad a establecer para cada motor
        # segun las velocidades lineal y angular deseadas
        im0 = np.array([[1/self.R, self.L/(2*self.R)],
                        [1/self.R, (-self.L)/(2*self.R)]])
        #print("im0", im0)
        im1 = np.array([v, np.deg2rad(w)])
        inverse_model = np.dot(im0, im1)
        #print("inverse_model", inverse_model)
        wd = inverse_model[0]
        wi = inverse_model[1]

        speedDPS_right = np.rad2deg(wd)
        speedDPS_left = np.rad2deg(wi)

        # Establece las velocidades de los motores con los valores calculados
        self.BP.set_motor_dps(self.BP.PORT_B, speedDPS_right)
        self.BP.set_motor_dps(self.BP.PORT_C, speedDPS_left)

        # Actualizacion de la odometria
        self.lock_odometry.acquire()
        # SC
        self.v.value = v
        self.w.value = np.deg2rad(w)
        self.lock_odometry.release()

    def readSpeed(self):
        """ Devuelve la velocidad lineal y angular actual del robot """

        self.lock_odometry.acquire()
        # SC
        [rightEngine, leftEngine] = [self.BP.get_motor_encoder(self.BP.PORT_B),
                                     self.BP.get_motor_encoder(self.BP.PORT_C)]

        # Se obtiene lo que ha girado cada rueda en esta iteracion (recorrido hasta ahora - recorrido anterior)
        deg_right_e = (rightEngine - self.acum_d) / self.P
        deg_left_e = (leftEngine - self.acum_i) / self.P

        # Actualizacion del anterior giro acumulado por las ruedas
        self.acum_d = rightEngine
        self.acum_i = leftEngine
        grados_ruedas = np.array(
            [[np.deg2rad(deg_right_e)], [np.deg2rad(deg_left_e)]])

        # Traccion diferencial
        trac = np.array(
            [[self.R/2, self.R/2], [self.R/self.L, (-self.R/self.L)]])

        # Calculo de las velocidades lineal y angular
        vel = np.dot(trac, grados_ruedas)

        self.lock_odometry.release()

        return vel[0], vel[1]

    def readOdometry(self):
        """ Returns current value of odometry estimation """

        self.lock_odometry.acquire()
        # SC
        x = self.x.value
        y = self.y.value
        th = self.th.value
        self.lock_odometry.release()
        return x, y, th

    def startOdometry(self):
        """ This starts a new process/thread that will be updating the odometry periodically """

        self.finished.value = False
        self.BP.offset_motor_encoder(self.BP.PORT_B,
                                     self.BP.get_motor_encoder(self.BP.PORT_B))  # reset encoder B
        self.BP.offset_motor_encoder(self.BP.PORT_C,
                                     self.BP.get_motor_encoder(self.BP.PORT_C))  # reset encoder C

        self.p = Process(target=self.updateOdometry, args=())
        self.p.start()
        print("PID: ", self.p.pid)

    # You may want to pass additional shared variables besides the odometry values and stop flag
    def updateOdometry(self):
        """ Actualiza la odometria con una frecuencia establecidad por el perido P """

        # Fichero de log
        if os.path.exists("log_odometry.log"):
            os.remove("log_odometry.log")
        log = open("log_odometry.log", "a")

        while not self.finished.value:
            # current processor time in a floating point value, in seconds
            tIni = time.clock()

            # Lee los valores reales de la velocidad lineal y angular
            [real_v, real_w] = self.readSpeed()

            # Calcula los nuevos valores de la odometria
            if real_w == 0:
                d_x = (real_v * self.P) * np.cos(self.th.value)
                d_y = (real_v * self.P) * np.sin(self.th.value)
                d_th = 0
            else:
                # El radio se calcula R = v/w
                d_th = real_w * self.P
                d_s = (real_v/real_w) * d_th
                d_x = d_s * np.cos(self.th.value + (d_th/2))
                d_y = d_s * np.sin(self.th.value + (d_th/2))

            # Actualiza la odometria con los nuevos valores en exclusion mutua
            self.lock_odometry.acquire()
            # SC
            self.x.value += d_x
            self.y.value += d_y
            self.th.value += d_th
            self.th.value = self.normalizar(self.th.value)
            self.lock_odometry.release()

            # Escribe en el LOG los valores actualizados de la odometria
            [x, y, th] = self.readOdometry()
            self.lock_odometry.acquire()
            # SC
            coord = str(x) + ',' + str(y) + ',' + str(th) + '\n'
            log.write(coord)
            self.lock_odometry.release()

            tEnd = time.clock()
            #time.sleep(self.P - (tEnd-tIni))

        # Escribe en el LOG los valores finales de la odometria
        [x, y, th] = self.readOdometry()
        self.lock_odometry.acquire()
        # SC
        coord = str(x) + ',' + str(y) + ',' + str(th) + '\n'
        log.write(coord)
        self.lock_odometry.release()

        #print("Stopping odometry ... X= %d" %(self.x.value))
        # sys.stdout.write("Stopping odometry ... X=  %.2f, \
        #        Y=  %.2f, th=  %.2f \n" %(self.x.value, self.y.value, self.th.value))

    def stopOdometry(self):
        """ Stop the odometry thread. """

        self.finished.value = True

        self.BP.reset_all()
        self.setSpeed(0, 0)

    def normalizar(self, th):
        """ Funcion de normalizacion del angulo entre -pi, pi """
        if th > math.pi:
            th = th - 2 * math.pi
        elif th < -math.pi:
            th = th + 2 * math.pi
        return th

    def getPeriod(self):
        """ Devuelve el periodo de actualizacion de la odometria """
        return self.P

    def trackObject(self, colorRangeMin=[0,0,0], colorRangeMax=[255,255,255]):
        #targetSize=??, target??=??, catch=??, ...)
        # NO SE QUE ES 
        targetSize = 20   #tamanyo dl target
        targetRojo = np.array([10,10,0])  #Posicion del target
        catch = False  #Si se coge el target o no
        finished = False
        targetFound = False
        targetPositionReached = False
        almost_centered = False
        # Inicializar la camara del robot 
        cam = cv2.VideoCapture(0)
        #cam.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        #cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        
        # allow the camera to warmup
        time.sleep(0.1)
        
        while not finished:
            tIni = time.clock()

            # 1. search the most promising blob
            _, imgBGR = cam.read() 
            
            blob = getRedBloobs(imgBGR) 
            w = 0.0
            v = 0.0
            if (blob is not None and almost_centered):

                if blob.size > 120:
                    finished = True
                #Se detecta la pelota
                #Se detecta la pelota pero es necesario que este en el centro
                if blob.pt[0] > imgBGR.cols():
                    #Se tiene que mover a la derecha
                    w = -20.0
                else: 
                    if blob.pt[0] < imgBGR.cols():
                        w = 20.0

                if blob.size < 40:
                    v = 45
                else:
                    if blob.size < 80:
                        v = 30
                    else:
                        v = 15
                
                self.setSpeed(v,w)        
                

            else:
                # Si no se ha encontrado la pelota en la imagen se comienza a girar para buscar la pelota
                if (blob is not None):
                    mid_img = imgBGR.cols()/2
                    if abs(blob.pt[0] - mid_img) < 20:
                        almost_centered = True
                    elif blob.pt[0] - mid_img < 100:
                        self.setSpeed(0,-10)
                    elif abs(blob.pt[0] - mid_img) > 100:
                        self.setSpeed(0,10)
                else:
                    self.setSpeed(0,-30)
                
            
                #while not targetPositionReached:
                #    # 2. decide v and w for the robot to get closer to target position
                #    if (True):
                #        targetPositionReached = True
                #        finished = True
            
            
            tEnd = time.clock()
            #print(tEnd-tIni)
            #time.sleep(0.5 - (tEnd-tIni))
        return finished
        

    
    #def catch(self):
    #    # decide the strategy to catch the ball once you have reached the target
    #    position
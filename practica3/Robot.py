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
import picamera
from picamera.array import PiRGBArray

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
        self.BP.offset_motor_encoder(self.BP.PORT_A,
                                     self.BP.get_motor_encoder(self.BP.PORT_A))  # PINZAS

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
        self.x_b= Value('d', 0)
        self.y_b = Value('d', 0)
        self.size_b = Value('d', 0)
        self.is_blob = Value('b', False)


        self.rows = Value('i',0)
        self.cols = Value('i',0)

        # Variables de vision 
        
        

    def setSpeed(self, v, w):
        """ Funcion que establece la velocidad lineal del robot a v y la velocidad
            angular del robot a w """

        #print("setting speed to %.2f %.2f" % (v, w))

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

        # Escribe en el LOG los valores finales de la odometria
        [x, y, th] = self.readOdometry()
        self.lock_odometry.acquire()
        # SC
        coord = str(x) + ',' + str(y) + ',' + str(th) + '\n'
        log.write(coord)
        self.lock_odometry.release()


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
    # Esta funcion busca y se acerca al objeto hasta estar en p
    def trackObject(self, colorRangeMin=[0,0,0], colorRangeMax=[255,255,255]):
        finished = False    #Variable para determinar que el robot ha cogido la pelota
        triedCatch = False  #Variable para determinar si el robot va a iniciar la accion de coger la pelota
        targetPositionReached = False   #Variable para determinar si el robot ha iniciado el proceso de coger la pelota
        #Se inicia el proces concurrente que lee la camara
        self.pCam = Process(target=self.updateCamara, args=())
        self.pCam.start()
        #Se deja que se inicie la camara
        time.sleep(1)
        while not finished:
            # Si se ha detectado un blob
            if (self.is_blob.value):
                x_actual = self.x_b.value # Se obtiene la coordenada x en la que se encuentra

                # Si el diametro es mayor que 150 se inica el proceso de catch porque esta muy cerca del robot
                if self.size_b.value > 140 and not triedCatch and abs(self.x_b.value - self.cols.value/2) < 50:
                    targetPositionReached = True # Se indica que se ha alcanzado el objeto 
                    self.setSpeed(0,0)
                else:
                    #La pelota se ha detectado pero esta lejos
                    self.trackObjectSpeed(x_actual,self.cols.value)  
            else:
                # En caso en el que la pelota desaparezca de la imagen se inicia busqueda
                #print('Pierdo el blob')
                self.find_ball(80)
                
            # Si previamente se ha realizado un intento de coger se comprueba si la pelota esta en las pinzas
            if self.is_blob.value and triedCatch:
                x_bl = self.x_b.value
                y_bl = self.y_b.value

                # Si el centro del blob esta en la parte inferior centrada de la imagen se considera que esta cogido
                if self.size_b.value > 195 and abs(self.x_b.value - self.cols.value/2) < 50 and self.y_b.value > self.rows.value/2:
                    self.setSpeed(0,0)
                    finished = True
                    print('LO TENGO')
                    self.setSpeed(0,30)
                    time.sleep(3)
                else:
                    print('No se ve la pelota en las pinzas')
                    print('x',x_bl, ', y', y_bl)
                    print('blob size', self.size_b.value)
                    triedCatch = False

            # Si se ha alcanzado la pelota y no se ha capturado previamente
            if targetPositionReached and not finished and not triedCatch: 
                print('Entro a catch')
                self.catch() # Se inicia el proceso de captura 
                targetPositionReached = False
                triedCatch = True
                   
        return finished
    
    # Funcion utilizada para decidir la velocidad y direccion del robot
    # en funcion de donde se encuenta la pelota en la imagen  
    def trackObjectSpeed(self,x_actual,cols):
        #Se divide la imagen en 8 sectores verticales y en funcion del que se encuentre la pelota se aplica una velocidad u otra
        if self.size_b.value > 100 and (self.x_b.value - cols/2) > 50:
            v = 0
            w = -10
        elif self.size_b.value > 100 and (self.x_b.value - cols/2) < -50:
            v = 0
            w = 10
        elif((x_actual > (3*cols)/8) and x_actual <= cols/2) or (x_actual >= cols/2 and x_actual<= (5*cols)/8):
            #Sector central [(3*cols)/8,(5*cols)/8]
            w = 0
            v = self.speed_size(self.size_b.value)
        elif(x_actual >= (2*cols)/8 and x_actual <= (3*cols)/8):
            #Primer sector izquierda de 2*cols/8 hasta 3cols/8 (el pequeñito entre el que esta lejos y el del centro)
            v = self.speed_size(self.size_b.value)
            w = self.w_speed_size(v,10)
        elif(x_actual >= 0 and x_actual <= (2*cols)/8):
            #Sector mas alejado de la izquierda de [0,(2*cols)/8]
            v = self.speed_size(self.size_b.value)
            w = self.w_speed_size(v,20)         
        elif(x_actual >= (5*cols)/8 and x_actual <= (6*cols)/8):
            #Primer sector derecha [(5*cols)/8, (6*cols)/8]
            v = self.speed_size(self.size_b.value)
            w = self.w_speed_size(v,-10)   
        elif(x_actual > (6*cols)/8 and x_actual <= cols):
            #Sector mas alejado de derecha [(6*cols)/8, cols]
            v = self.speed_size(self.size_b.value)  
            w = self.w_speed_size(v,-20)             
        self.setSpeed(v,w)  

    # Funcion utilizada para decidir la velocidad lienal de acercamiento 
    # hacia la pelota en funcion de su tamanyo
    def speed_size(self,size):
        if (size < 100):
            return 150
        else:
            return 100   
    
    # Funcion utilizada para decidir la velocidad angular de acercamiento 
    # hacia la pelota en funcion de su tamanyo
    def w_speed_size(self,v,expected_w):
        if (v < 100):
            return expected_w
        else:
            return expected_w/2 
    
    # Funcion utilizada para decidir el lado de rotacion
    #Si la pelota se pierde por un lado, el robot la seguira por ese mismo lado
    # por defecto rotacion hacia la derecha 
    def find_ball(self, vel):
        mid_img = self.cols.value/2
        if self.x_b.value < mid_img:
            self.setSpeed(0, vel)
        else:
            self.setSpeed(0,-vel)

    # Funcion de captura de la pelota
    def catch(self):
        w = 40   # Velocidad angular para abrir las pinzas 
        self.BP.set_motor_dps(self.BP.PORT_A, w)
        time.sleep(1.5) # Tiempo de apertura 
        self.BP.set_motor_dps(self.BP.PORT_A, 0)
        self.setSpeed(60, 0)
        time.sleep(1.7) # Resto de acercamiento a la pelota
        self.setSpeed(0, 0)
        w = -42    # Velocidad angular para cerrar las pinzas 
        self.BP.set_motor_dps(self.BP.PORT_A, w)
        time.sleep(1.5) # Tiempo de cierre de pinzas
        self.BP.set_motor_dps(self.BP.PORT_A, 0)
    
    #Proceso concurrente que sirve para capturar imagenes
    #Se realiza un proceso concurrente para que la captura de imagenes sea mas rapida y eficiente
    def updateCamara(self):
        #Se inicia la camara del robot
        cam = picamera.PiCamera()
        cam.resolution = (640,480)
        cam.framerate = 32 
        rawCapture = PiRGBArray(cam, size=(640, 480))
        #Se espera un tiempo para que se pueda iniciar la camara
        time.sleep(0.1)
        # Se captura una imagen inicial para obtener el tamanyo de la imagen 
        self.rows.value = 480
        self.cols.value = 640
        #Mientras no se detengaa el robot, se siguen captando imagenes
        while not self.finished.value:
            cam.capture(rawCapture, format="bgr", use_video_port=True)
            # clear the stream in preparation for the next frame
            rawCapture.truncate(0)
            frame = rawCapture.array
            blob = getRedBloobs(frame)  # Se devuelve el blob mas grande
            #Se actualizan las variables compartidas referentes a la imagen
            if blob is not None:
                self.x_b.value = blob.pt[0]
                self.y_b.value = blob.pt[1]
                self.size_b.value = blob.size 
                self.is_blob.value = True
            else:
                self.is_blob.value = False

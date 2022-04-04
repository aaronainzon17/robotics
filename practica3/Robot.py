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
        finished = False
        triedCatch = False
        targetPositionReached = False
        almost_centered = False
        last_bloob = None # Se alamcena la ultima imagen en la que aparece un bloob 
        
        # Inicializar la camara del robot 
        cam = cv2.VideoCapture(0)
        #cam.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        #cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        
        # Inicializacion de la camara
        time.sleep(0.1)
        
        while not finished:
            # Busqueda del blob mas prometedor 
            _, frame = cam.read()       # Se captura un fotograma
            blob = getRedBloobs(frame)  # Se devuelve el blob mas grande
            
            rows,cols,_ = frame.shape   # Se obtiene el numero de filas y columnas del fotograma 
            
            # Si se ha detectado un blob casi centrado en la imagen
            if (blob is not None and almost_centered):
                x_actual = blob.pt[0] # Se obtiene la coordenada x en la que se encuentra 
                # Si el diametro es mayor que 120 se inica el proceso de catch
                if blob.size > 120 and not triedCatch:
                    print('Paro porque he encontrado un blob de', blob.size)
                    targetPositionReached = True # Se indica que se ha alcanzado el objeto 
                    #self.setSpeed(0,0) CREO QUE SE PUEDE BORRAR 

                self.trackObjectSpeed(x_actual,cols,blob)  
                  
            else:
                mid_img = cols/2 # Se calcula en eje central de la imagen

                # Si se ha encontrado la pelota en la imagen se ralentiza el giro hasta centrarla
                if (blob is not None):  
                    last_bloob = blob
                    self.find_ball(last_bloob, 20, mid_img) # Se ralentiza el giro PREV: self.setSpeed(0,-20)
                    # Si se encuentra a 80 pixeles del centro 
                    if abs(blob.pt[0] - mid_img) < 80:
                        almost_centered = True # Se indica que el blob esta casi centrado
                else:
                    # Si no se ha encontrado la pelota en la imagen se comienza a girar para buscar la pelota
                    self.find_ball(last_bloob, 40, mid_img) # PREV = self.setSpeed(0,-40)
                    almost_centered = False

            # Si previamente se ha realizado un intento de coger se comprueba si la pelota esta en las pinzas
            if (blob is not None and triedCatch):
                x_bl,y_bl = [blob.pt[0],blob.pt[1]]
                print('tengo y en',y_bl, 'y x en',x_bl)
                print('x es',5*rows/6, 'y el centro ', abs(x_bl - cols/2))
                
                # Si el centro del blob esta en la parte inferior centrada de la imagen se considera que esta cogido
                if (blob.size > 215 and abs(y_bl - 350) < 20) and (abs(x_bl - cols/2) < 20):
                    finished = True
                    print('LO TENGOOO :)')
                    #
                else:
                    print('No se ve la pelota en las pinzas')
                    triedCatch = False
            
            # Si se ha alcanzado la pelota y no se ha capturado previamente
            if targetPositionReached and not finished: 
                print('Entro a catch')
                self.catch() # Se inicia el proceso de captura 
                targetPositionReached = False
                triedCatch = True
                   
        return finished
    
    # Funcion utilizada para decidir la velocidad y direccion del robot
    # en funcion de donde se encuenta la pelota en la imagen  
    def trackObjectSpeed(self,x_actual,cols,blob):
        #Se divide la imagen en 8 sectores verticales y en funcion del que se encuentre la pelota se aplica una velocidad u otra
        if((x_actual > (3*cols)/8) and x_actual <= cols/2) or (x_actual >= cols/2 and x_actual<= (5*cols)/8):
            #Sector central [(3*cols)/8,(5*cols)/8]
            w = 0
            v = self.speed_size(blob.size)
        elif(x_actual >= (2*cols)/8 and x_actual <= (3*cols)/8):
            #Primer sector izquierda de 2*cols/8 hasta 3cols/8 (el pequeñito entre el que esta lejos y el del centro)
            v = self.speed_size(blob.size)
            w = self.w_speed_size(v,10)
        elif(x_actual >= 0 and x_actual <= (2*cols)/8):
            #Sector mas alejado de la izquierda de [0,(2*cols)/8]
            v = self.speed_size(blob.size)
            w = self.w_speed_size(v,20)
        elif(x_actual >= (5*cols)/8 and x_actual <= (6*cols)/8):
            #Primer sector derecha [(5*cols)/8, (6*cols)/8]
            v = self.speed_size(blob.size)
            w = self.w_speed_size(v,-10)  
        elif(x_actual > (6*cols)/8 and x_actual <= cols):
            #Sector mas alejado de derecha [(6*cols)/8, cols]
            v = self.speed_size(blob.size)  
            w = self.w_speed_size(v,-20)
                    
        self.setSpeed(v,w)  

    # Funcion utilizada para decidir la velocidad lienal de acercamiento 
    # hacia la pelota en funcion de su tamanyo
    def speed_size(self,size):
        if (size < 100):
            return 150
        else:
            return 60   
    
    # Funcion utilizada para decidir la velocidad angular de acercamiento 
    # hacia la pelota en funcion de su tamanyo
    def w_speed_size(self,v,expected_w):
        if (v < 60):
            return expected_w
        else:
            return expected_w/2 
    
    # Funcion utilizada para decidir el lado de rotacion
    # por defecto rotacion hacia la derecha 
    def find_ball(self, last_blob, vel, mid_img):
        if last_blob is not None:
            x_lb = last_blob.pt[0]
            if x_lb < mid_img:
                self.setSpeed(0, vel)
            else:
                self.setSpeed(0,-vel)
        else:
            self.setSpeed(0,-vel)

    # Funcion de captura de la pelota
    def catch(self):
        w = 40   # Velocidad angular para abrir las pinzas 
        self.BP.set_motor_dps(self.BP.PORT_A, w)
        time.sleep(1.5) # Tiempo de apertura en movimento 
        self.BP.set_motor_dps(self.BP.PORT_A, 0)
        time.sleep(0.6) # Resto de acercamiento a la pelota
        self.setSpeed(0, 0)
        w = -42    # Velocidad angular para cerrar las pinzas 
        self.BP.set_motor_dps(self.BP.PORT_A, w)
        time.sleep(1.5) # Tiempo de cierre de pinzas
        self.BP.set_motor_dps(self.BP.PORT_A, 0)



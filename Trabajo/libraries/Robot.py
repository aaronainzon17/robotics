#!/usr/bin/python
# -*- coding: UTF-8 -*-
# use python 3 syntax but make it compatible with python 2
from __future__ import print_function
from __future__ import division
from ast import alias

# import brickpi3 # import the BrickPi3 drivers
import time

from cv2 import distanceTransformWithLabels     # import the time library for the sleep function
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

sys.path.append('../libraries')
from BlobDetector import getRedBloobs, detect_red, getGreenBloobs, detect_green
from sample_matching import match_images


class Robot:
    def __init__(self, init_position=[0.0, 0.0, 0.0]):
        """
        Initialize basic robot params. \

        Initialize Motors and Sensors according to the set up in your robot
        """

######## UNCOMMENT and FILL UP all you think is necessary (following the suggested scheme) ########

        # Robot construction parameters

        # Radio de la rueda y distancia entre ruedas
        self.R = 26 #28
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
        self.BP.set_sensor_type(self.BP.PORT_1,
                                    self.BP.SENSOR_TYPE.EV3_ULTRASONIC_CM)  # ULTRASONIDOS
        self.BP.set_sensor_type(self.BP.PORT_2,
                                    self.BP.SENSOR_TYPE.NXT_LIGHT_ON)  # SENSOR DE LUZ
        self.BP.set_sensor_type(self.BP.PORT_4,
                                    self.BP.SENSOR_TYPE.EV3_GYRO_ABS_DPS) # GIROSCOPIO
        self.BP.set_sensor_type(self.BP.PORT_3,
                            self.BP.SENSOR_TYPE.NXT_ULTRASONIC) #ULTRASONIDOS SEGUIMIENTO PARED

        ##################################################

        # odometry shared memory values
        self.x = Value('d', init_position[0])
        self.y = Value('d', init_position[1])
        self.th = Value('d', init_position[2])
        # boolean to show if odometry updates are finished
        self.finished = Value('b', 1)
        self.finished_capture_green=Value('b',1)

        # if we want to block several instructions to be run together, we may want to use an explicit Lock
        self.lock_odometry = Lock()

        # odometry update period
        self.P = 0.07
        
        # Blob values
        self.x_b= Value('d', 0)
        self.y_b = Value('d', 0)
        self.size_b = Value('d', 0)
        self.is_blob = Value('b', False)
        self.red_pixels = Value('i', 0)


        self.rows = Value('i',0)
        self.cols = Value('i',0)

        # Variables de vision
        self.mapa = None
        self.img_salida = None
        self.img_NO_salida = None
        self.casilla_salida = None 
        

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
        th_ini = self.th.value
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
                #abs_th = self.norm_pi(th_ini + self.read_gyros()) #self.read_gyros()
                
                # El radio se calcula R = v/w
                d_th = real_w * self.P #self.norm_pi(abs_th - self.th.value) #
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
            time.sleep(self.P - (tEnd - tIni))

        # Escribe en el LOG los valores finales de la odometria
        [x, y, th] = self.readOdometry()
        self.lock_odometry.acquire()
        # SC
        coord = str(x) + ',' + str(y) + ',' + str(th) + '\n'
        log.write(coord)
        self.lock_odometry.release()

    def read_gyros(self):
        arr = []
        for i in range(5):
            arr.append(self.BP.get_sensor(self.BP.PORT_4)[0] *-1) 
        #print(arr)
        return self.norm_pi(np.deg2rad(np.median(arr)))

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
                if self.size_b.value > 220 and not triedCatch and abs(self.x_b.value - self.cols.value/2) < 50:
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
                if self.red_pixels.value > 300:
                    self.setSpeed(0,0)
                    finished = True
                else:
                    print('No se ve la pelota en las pinzas')
                    print('x',x_bl, ', y', y_bl)
                    print('blob size', self.size_b.value)
                    triedCatch = False
                    self.uncatch()

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
        if self.size_b.value > 160 and (self.x_b.value - cols/2) > 50:
            v = 0
            w = -10
        elif self.size_b.value > 160 and (self.x_b.value - cols/2) < -50:
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
        w = 150 #40   # Velocidad angular para abrir las pinzas 
        #Que avance un poquito antes de bajar la pinza
        self.setSpeed(80,0)
        time.sleep(0.7)
        self.setSpeed(0,0)
        self.BP.set_motor_dps(self.BP.PORT_A, w)
        time.sleep(0.6) # Bajar cesta
        self.BP.set_motor_dps(self.BP.PORT_A, 0)
        
        

    def uncatch(self):
        w = -44 #-42    # Velocidad angular para cerrar las pinzas 
        self.BP.set_motor_dps(self.BP.PORT_A, w)
        time.sleep(2) # Tiempo de cierre de pinzas
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
            self.red_pixels.value = detect_red(frame)
            #Se actualizan las variables compartidas referentes a la imagen
            if blob is not None:
                self.x_b.value = blob.pt[0]
                self.y_b.value = blob.pt[1]
                self.size_b.value = blob.size 
                self.is_blob.value = True
            else:
                self.is_blob.value = False

            # Se utiliza la camara para detectar la casilla de salida
            #self.detectar_casilla_salida(frame)
    
    def updateCamaraGreen(self):
        self.finished_capture_green.value=False
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
        while not self.finished_capture_green.value:
            cam.capture(rawCapture, format="bgr", use_video_port=True)
            # clear the stream in preparation for the next frame
            rawCapture.truncate(0)
            frame = rawCapture.array
            blob = getGreenBloobs(frame)  # Se devuelve el blob mas grande
            #self.red_pixels.value = detect_red(frame)
            #Se actualizan las variables compartidas referentes a la imagen
            if blob is not None:
                self.x_b.value = blob.pt[0]
                self.y_b.value = blob.pt[1]
                self.size_b.value = blob.size 
                self.is_blob.value = True
            else:
                self.is_blob.value = False
    
    def setNewPosition(self,x_new, y_new, th_new):
        self.x.value = x_new
        self.y.value = y_new
        self.th.value = th_new

    def go(self, x_goal, y_goal):
        # Aliena al robot con el siguiente punto
        self.align(x_goal, y_goal, np.deg2rad(1))
        # Se le asigna una velocidad lienal
        self.setSpeed(120,0)
        # Se comprueba que el robot alcanza correctamente la posicion 
        self.check_position(x_goal, y_goal, 25, 25)
   
    # check_position es la funcion de control de localizacion
    # En ella se comprueba la posicion real del robot leida de los
    # motores y se comprueba si se encuentra en la posicion deseada
    # permitiendo un cierto error.
    def check_position(self, x_goal, y_goal, x_err, y_err):
        reached = False
        while not reached:
            [x_now, y_now, th_now] = self.readOdometry()
            # Se comprueba que la trayectoria se esta relaizando dentro del error permitido
            if (abs(x_goal-x_now) <= x_err) and (abs(y_goal-y_now) <= y_err):
                self.setSpeed(0,0)
                reached = True
                
    # La funcion se encarga de alienar el robot con el punto objetivo para poder
    # realizar una trayectoria lienal
    def align(self, x_goal, y_goal, error_ang):
            aligned = False 
            
            while not aligned:
                [x_now, y_now, th_now] = self.readOdometry()
                d_x = x_goal - x_now
                d_y = y_goal - y_now
                d_th = self.norm_pi(np.arctan2(d_y, d_x) - th_now)
                if abs(d_th) < error_ang:
                    self.setSpeed(0,0)
                    
                    aligned = True
                else:
                    w = self.lienar_w(d_th)
                    self.setSpeed(0,w)
    
    # Funcion que define la velocidad angular en funcion de los 
    # grados restantes w = [10,90]
    def lienar_w(self,dth):
        w = dth*(80/math.pi)
        if w < 0:
            w -= 10
        else:
            w+= 10
        return w

    #Funcion que normaliza el angulo entre -pi, pi
    def norm_pi(self, th):
        if th > math.pi:
            th = th - 2 * math.pi
        elif th < -math.pi:
            th = th + 2 * math.pi
        return th

    # Se evalua si el sensor ultrasonidos ha detectado un obstaculo a menos de
    # la distancia que se va a recorrer 
    def detectObstacle(self, x_goal, y_goal):
        # Aliena al robot con el siguiente punto
        self.align(x_goal, y_goal, np.deg2rad(1))
        
        value = 0.0
        # Se itera hasta que el sensor devuelve un valor distinto a 0 (le cuesta encenderse y sino da error)
        while value <= 0.0:
            try:
                
                [x_now, y_now, _] = self.readOdometry()
                # Se va a definir ,un umbral para detectar el obsatculo en  
                # el borde de la celda o en el centro de la celda objetivo
                wall_o = 30 * 10 # mm
                center_o = 50 * 10 # mm
                # Se lee la distancia que recoge el sensor (*10 para pasarlo a mm)
                value = self.BP.get_sensor(self.BP.PORT_1) * 10
                
                # Se devuelve True si hay obstaculo, False si no
                if value < wall_o and value > 0.0:
                    return [True, True] # Hay obstaculo y esta en la pared
                elif value > wall_o and value < center_o and value > 0.0:
                    return [True, False] # Hay obsaculo y no esta en la pared
                elif value > 0.0:
                    return [False, False] # No hay obstaculo 

            except brickpi3.SensorError as error:
                print(error) 
        
    def detectar_recorrido(self):
        value = 0.0
        while value <= 2000:
            try:
                value = self.BP.get_sensor(self.BP.PORT_2)
                #print('He leido', value)
                if value > 2000 and value < 2750:     #Se detecta el colo blanco entonces es el A
                    # Se indican las coordenadas iniciales del robot 
                    self.x = Value('d', 600)
                    self.y = Value('d', 2800)
                    self.th = Value('d', self.norm_pi(np.deg2rad(-90)))
                    # Se indican las imagenes a detectar
                    self.img_salida = './R2-D2_s.png'
                    self.img_NO_salida = './BB8_s.png'
                    self.mapa = 'A'
                    return "mapaA_CARRERA.txt"
                elif value > 2000 :
                    # Se indican las coordenadas iniciales del robot 
                    self.x = Value('d', 2200)
                    self.y = Value('d', 2800)
                    self.th = Value('d', self.norm_pi(np.deg2rad(-90)))
                    # Se indican las imagenes a detectar
                    self.img_salida = './BB8_s.png'
                    self.img_NO_salida = './R2-D2_s.png'
                    self.mapa = 'B'
                    return "mapaB_CARRERA.txt"
                else:
                    return None
            except brickpi3.SensorError as error:
                print(error)
            time.sleep(0.02)

    # Esta funcion busca y se acerca al objeto hasta estar en p
    def detect_scape(self):
        # Si la salida no se ha encontrado
        if self.casilla_salida is None:
            cam = picamera.PiCamera()

            #cam.resolution = (320, 240)
            cam.resolution = (640, 480)
            cam.framerate = 10 # less frame rate, more light BUT needs to go slowly (or stop)
            rawCapture = PiRGBArray(cam)
            
            # allow the camera to warmup
            time.sleep(0.2)

            # Se determinan puntos clave del mapa para ver los robots
            if self.mapa == 'A':
                imgs_center = [2000,2800]
                center_table = [2000,2000]
            else:
                imgs_center = [800, 2800]
                center_table = [800,2000]
            
            # Se buscan las imagenes
            self.go(center_table[0],center_table[1])
            self.align(imgs_center[0], imgs_center[1], np.deg2rad(1))

            cam.capture(rawCapture, format="bgr")
            frame = rawCapture.array 
            
            self.detectar_casilla_salida(frame)

            rawCapture.truncate(0)
        
        # Si no lo ha encontardo yendo al centro del mapa se rota para buscar
        x_face = imgs_center[0]
        while self.casilla_salida is None and x_face > (imgs_center[0] - 400):
            x_face -= 200
            self.align(x_face, imgs_center[1], np.deg2rad(1))
            
            cam.capture(rawCapture, format="bgr")
            frame = rawCapture.array 
            
            self.detectar_casilla_salida(frame)

            rawCapture.truncate(0)
        
        x_face = imgs_center[0]
        while self.casilla_salida is None and x_face < (imgs_center[0] < 400):
            x_face += 200
            self.align(x_face, imgs_center[1], np.deg2rad(1))
            
            cam.capture(rawCapture, format="bgr")
            frame = rawCapture.array 
            
            self.detectar_casilla_salida(frame)

            rawCapture.truncate(0)

        #cam.close()
        print('Salgo por la casilla', self.casilla_salida)
        
        # Una vez se ha encontrado la salida se sale
        #self.go(self.casilla_salida[0],self.casilla_salida[1])
        #self.go(self.casilla_salida[0],(self.casilla_salida[1] + 400))

    def detectar_casilla_salida(self, frame):
        if self.casilla_salida == None: 
            found_salida, x_salida = match_images(self.img_salida, frame)
            found_NO_salida, x_NO_salida = match_images(self.img_NO_salida, frame)
            if found_salida and found_NO_salida and self.mapa == 'A':
                if x_salida < x_NO_salida:
                    self.casilla_salida = [1400,2600] # [3,6]
                else:
                    self.casilla_salida = [2600,2600] # [6,6]
            if found_salida and found_NO_salida and self.mapa == 'B':
                if x_salida < x_NO_salida:
                    self.casilla_salida = [200,2600] # [0,6]
                else:
                    self.casilla_salida = [1400,2600] # [3,6]
        else:
            print('He detectado la salida en:', self.casilla_salida)

    def relocalizarRobot(self):
        #El robot esta de frente a una pared
        #Toma una imagen
        #saca la nueva x,y,th
        #Resetea los encoders
        self.lock_odometry.acquire()
        #Se usa el sonar para centrarse
        
        
        self.BP.offset_motor_encoder(self.BP.PORT_B,
                                     self.BP.get_motor_encoder(self.BP.PORT_B))  # reset encoder B
        self.BP.offset_motor_encoder(self.BP.PORT_C,
                                     self.BP.get_motor_encoder(self.BP.PORT_C))  # reset encoder C

        self.lock_odometry.release()


    def centrar_con_imagen(self):
        #Idea:
        #Hacer regla de tres
        #x = (a*b)/c
        #a=tBlob=120 , b=yRobot=1000 , c=tBlob'=120 ->x=yRobot'=952.3809
        #a=xBlob=320 , b=xRobot=600 , c=xBlob'=300 ->x=xRobot'=
        #Mejor retomar las medidas
        t_prueba=120    #Variables para sacar la regla de 3 de y
        y_prueba=1000   #Variables para sacar la regla de 3 de y
        x_prueba=320
        x_robot_prueba=600
        maxFotos=10
        numFotos = 0
        tamanyo_blob=0
        pos_blob=0
        #Empezar proceso de la camara
        self.pCam = Process(target=self.updateCamaraGreen, args=())
        self.pCam.start()
        time.sleep(2)
        while(numFotos!=maxFotos):
            #Tomar foto sacar el blob
            if(self.is_blob.value):
                numFotos+=1
                tamanyo_blob+=self.size_b.value
                pos_blob+=self.x_b.value

        self.finished_capture_green=True
        tamanyo_blob=tamanyo_blob/maxFotos
        pos_blob=pos_blob/maxFotos
        y_actual = (t_prueba*y_prueba)/tamanyo_blob     #Esta es la y actual del robot
        x_actual = (x_prueba*x_robot_prueba)/pos_blob
        #Ahora se resetean los motores
        self.lock_odometry.acquire()
        # SC
        self.BP.offset_motor_encoder(self.BP.PORT_B,
                                     self.BP.get_motor_encoder(self.BP.PORT_B))  # reset encoder B
        self.BP.offset_motor_encoder(self.BP.PORT_C,
                                     self.BP.get_motor_encoder(self.BP.PORT_C))  # reset encoder C
        self.x.value=x_actual
        self.y.value=y_actual
        self.lock_odometry.release()
        odom = self.readOdometry()
        print("EL tamano del blob en centrar es ",tamanyo_blob)
        print(odom)
    
    def seguimientoPared(self, dc):
        # PARÁMETROS
        k1 = 0.00015  # estabilizar
        k2 = -0.04  # amortiguar


        wmax = np.deg2rad(45)
        reached = False
        # velocidad lineal constante
        v = 100  # mm/s


        # Medida del sensor
        d = 0
        while d == 0: 
            try:
                d = self.BP.get_sensor(self.BP.PORT_3)
                print(d)
            except brickpi3.SensorError as error:
                print(error) 
        d_ant = d

        # Bulce de seguimiento de la pared
        while not reached:
            # se calcula la velocidad angular
            w = k1*(dc-d)+k2*(d-d_ant)
            if w > 0:
                wc = min(wmax, w)
            else:
                wc = max(-wmax, w)

            self.setSpeed(v, wc)
            time.sleep(0.2)

            [x,_,_] = self.readOdometry()
            # Se comprueba que se ha alcanzado la salida
            if x > 2800:
                print('Se detiene porque llega')
                self.setSpeed(0,0)
                reached = True
            d_ant = d

            # Medida del sensor
            try:
                d = self.BP.get_sensor(self.BP.PORT_3)
                print(d)
            except brickpi3.SensorError as error:
                print(error) 


        
        
        
        





#########################################################
################## Funciones obsoletas ##################
#########################################################

    def is_lienar_mov(self, x_act, y_act, x_goal, y_goal):
        if abs(x_act - x_goal) < 100:
            # Movimiento lineal en y
            return True
        elif  abs(y_act - y_goal) < 100:
            # Movimiento lienal en x
            return True
        else:
            # Movimiento cirular
            return False

    
    
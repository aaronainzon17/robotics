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
import statistics
from picamera.array import PiRGBArray

# tambien se podria utilizar el paquete de threading
from multiprocessing import Process, Value, Array, Lock

sys.path.append('../libraries')
from BlobDetector import getRedBloobs, detect_red
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
        self.L = 129 #128

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
        self.th_ini = Value('d', init_position[2])
        
        # boolean to show if odometry updates are finished
        self.finished = Value('b', 1)
        self.parar_camara = Value('b', False)

        # if we want to block several instructions to be run together, we may want to use an explicit Lock
        self.lock_odometry = Lock()

        # odometry update period
        self.P = 0.04
        
        # Blob values
        self.x_b = Value('d', 0)
        self.y_b = Value('d', 0)
        self.size_b = Value('d', 0)
        self.is_blob = Value('b', False)
        self.red_pixels = Value('i', 0)

        #Image size shared data
        self.rows = Value('i',0)
        self.cols = Value('i',0)

        # Variables de recorrido
        self.mapa = None
        self.img_salida = None
        self.img_NO_salida = None
        self.casilla_salida = None 

        # Fichero de log odometria
        self.log = []

        

    def setSpeed(self, v, w):
        """ Funcion que establece la velocidad lineal del robot a v y la velocidad
            angular del robot a w """


        # Calculo de la velocidad a establecer para cada motor
        # segun las velocidades lineal y angular deseadas
        im0 = np.array([[1/self.R, self.L/(2*self.R)],
                        [1/self.R, (-self.L)/(2*self.R)]])
        
        im1 = np.array([v, np.deg2rad(w)])
        inverse_model = np.dot(im0, im1)
        
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

        #self.lock_odometry.acquire()
        # SC
        [rightEngine, leftEngine] = [self.BP.get_motor_encoder(self.BP.PORT_B),
                                     self.BP.get_motor_encoder(self.BP.PORT_C)]

        # Se obtiene lo que ha girado cada rueda en esta iteracion (recorrido hasta ahora - recorrido anterior)
        deg_right_e = rightEngine - self.acum_d
        deg_left_e = leftEngine - self.acum_i

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

        #self.lock_odometry.release()

        return vel[0], vel[1], np.deg2rad(deg_right_e), np.deg2rad(deg_left_e)

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

    # You may want to pass additional shared variables besides the odometry values and stop flag
    def updateOdometry(self):
        """ Actualiza la odometria con una frecuencia establecidad por el perido P """
        
        prev_gyros = 0.0

        while not self.finished.value:
            # current processor time in a floating point value, in seconds
            tIni = time.clock()

            # Lee los valores reales de la velocidad lineal y angular
            real_v, real_w, deg_right_e, deg_left_e  = self.readSpeed()
            
            # Calcula los nuevos valores de la odometria
            #if real_w == 0:
            #    d_x = (real_v * self.P) * np.cos(self.th.value)
            #    d_y = (real_v * self.P) * np.sin(self.th.value)
            #    d_th = 0
            #else:
            gyros_now = self.read_gyros() #self.read_gyros()
            
            # El radio se calcula R = v/w
            d_th = self.norm_pi(gyros_now - prev_gyros)
            #print("Diferencial de th ",d_th,"Grados leidos del giroscopio",gyros_now)
            #d_s = (real_v/real_w) * d_th
            d_s = (deg_right_e + deg_left_e)/2 * self.R
            d_x = d_s * np.cos(self.th.value + (d_th/2))
            d_y = d_s * np.sin(self.th.value + (d_th/2))
            prev_gyros = gyros_now

            # Actualiza la odometria con los nuevos valores en exclusion mutua
            self.lock_odometry.acquire()
            
            self.x.value += d_x
            self.y.value += d_y
            self.th.value += d_th   
            self.th.value = self.norm_pi(self.th.value)  
            
            self.lock_odometry.release()
            
            #Se escribe la odometria
            #self.write_log()

            tEnd = time.clock()
            elapsed = self.P - (tEnd - tIni)
            if elapsed >= 0:
                time.sleep(elapsed)
            else:
                print('No sleep')

        # Escribe en el LOG los valores finales de la odometria
        self.write_log()


    def write_log(self):
        """Funcion que escribe la odometria actual en el fichero de log"""
        [x, y, th] = self.readOdometry()
        self.lock_odometry.acquire()
        # SC
        coord = str(x) + ',' + str(y) + ',' + str(th) + '\n'
        self.log.append(coord)
        self.lock_odometry.release()

    def save_log(self):
        # Fichero de log
        if os.path.exists("log_odometry.log"):
            os.remove("log_odometry.log")
        log = open("log_odometry.log", "a")

        for pos in self.log:
            log.write(pos)

    def read_gyros(self):
        """Se leen los datos del giroscopo"""
        arr = []
        for i in range(5):
            try:
                arr.append(self.BP.get_sensor(self.BP.PORT_4)[0] *-1) 
            except brickpi3.SensorError as error:
                print(error) 
        return np.deg2rad(np.median(arr))

    def stopOdometry(self):
        """ Stop the odometry thread. """

        self.finished.value = True
        self.save_log()
        self.BP.reset_all()
        self.setSpeed(0, 0)

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
                if self.size_b.value > 200 and not triedCatch and abs(self.x_b.value - self.cols.value/2) < 50:
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
                if self.red_pixels.value > 170:
                    self.setSpeed(0,0)
                    finished = True
                    self.parar_camara.value = True
                    self.write_log()
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

        #time.sleep(1)           
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
        while not self.parar_camara.value:
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
        cam.close()
    
    def setNewPosition(self,x_new, y_new, th_new):
        """Permite actualizar la posicion del la odometria por si es necesario relocalizarse"""
        self.x.value = x_new
        self.y.value = y_new
        self.th.value = th_new

    def go(self, x_goal, y_goal, speed):
        # Alinea al robot con el siguiente punto
        self.align(x_goal, y_goal, np.deg2rad(1))
        self.write_log()
        _,_,th = self.readOdometry()
        if abs(abs(th) - np.deg2rad(90)) < np.deg2rad(5):
            x_err = np.Infinity
            y_err = 25
        elif abs(abs(th) - np.deg2rad(180)) < np.deg2rad(5) or abs(th) < np.deg2rad(5):
            x_err = 25
            y_err = np.Infinity
        else: 
            x_err = 50
            y_err = 50
        
        # Se le asigna una velocidad lienal
        self.setSpeed(speed,0)

        # Se comprueba que el robot alcanza correctamente la posicion 
        self.check_position(x_goal, y_goal, x_err, y_err)
   
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
    
    def check_position_3_values(self, x, y, th, x_err, y_err, angular_err):
        """ check_position es la funcion de control de localizacion
        En ella se comprueba la posicion real del robot leida de los
        motores y se comprueba si se encuentra en la posicion deseada
        permitiendo un cierto error. """

        # Se lee incialmente la posicion del robot
        [x_now, y_now, th_now] = self.readOdometry()
        reached = False

        while not reached:

            # Se calcula el angulo
            error_ang = abs(th-th_now)
            if error_ang > math.pi:
                error_ang = (2*math.pi) - error_ang

            # Se comprueba que la trayectoria se esta relaizando dentro del error permitido
            if (abs(x-x_now) <= x_err) and (abs(y-y_now) <= y_err) and (error_ang <= angular_err):
                reached = True
                print("Se ha alcanzado el punto:[", x_now, ",", y_now, ",", th_now, "]")
            else:
                [x_now, y_now, th_now] = self.readOdometry()

    def check_angle(self,th, angular_err):
        """ check_angle comprueba que la orientacion del robot es la correcta """

        # Se lee incialmente la posicion del robot
        [x_now, y_now, th_now] = self.readOdometry()
        reached = False

        while not reached:

            # Se calcula el angulo
            error_ang = abs(th-th_now)
            if error_ang > math.pi:
                error_ang = (2*math.pi) - error_ang

            # Se comprueba que la trayectoria se esta relaizando dentro del error permitido
            if error_ang <= angular_err:
                reached = True
                print("Se ha alcanzado el punto:[", x_now, ",", y_now, ",", th_now, "]")
                self.setSpeed(0,0)
            else:
                [x_now, y_now, th_now] = self.readOdometry()                
    
   
    def align(self, x_goal, y_goal, error_ang):
        """ La funcion se encarga de alienar el robot con el punto 
        objetivo para poder realizar una trayectoria lienal"""

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

    # # Se evalua si el sensor ultrasonidos ha detectado un obstaculo a menos de
    # la distancia que se va a recorrer 
    def detectObstacle(self, x_goal, y_goal):
        # Aliena al robot con el siguiente punto
        self.align(x_goal, y_goal, np.deg2rad(1))
        value = self.read_ultrasonyc()
        [x_now, y_now, _] = self.readOdometry()
        # Se calcula el espacio a recorrer 
        #espacio = np.linalg.norm([x_goal - x_now, y_goal - y_now])
        espacio = 250
        
        # Se devuelve True si hay obstaculo, False si no
        if value < espacio:
            print('Obstaculo detectado a', value)
            return True
        elif value > 0.0:
            return False
        
    def detectar_recorrido(self):
        value = 0.0
            
        arr = []
        for i in range(20):
            try:
                arr.append(self.BP.get_sensor(self.BP.PORT_2)) 
            except brickpi3.SensorError as error:
                print(error) 
        value = np.median(arr)
        
        print('He leido', value)
        if  value < 2750:     #Se detecta el colo blanco entonces es el A
            # Se indican las coordenadas iniciales del robot 
            self.x.value  = 600
            self.y.value = 2800
            self.th.value = self.norm_pi(np.deg2rad(-90))
            self.th_ini.value = self.th.value
            # Se indican las imagenes a detectar
            self.img_salida = './R2-D2_s.png'
            self.img_NO_salida = './BB8_s.png'
            self.mapa = 'A'
            return "mapaA_CARRERA.txt"
        else:
            # Se indican las coordenadas iniciales del robot 
            self.x.value = 2200
            self.y.value = 2800
            self.th.value = self.norm_pi(np.deg2rad(-90))
            self.th_ini.value = self.th.value
            # Se indican las imagenes a detectar
            self.img_salida = './BB8_s.png'
            self.img_NO_salida = './R2-D2_s.png'
            self.mapa = 'B'
            return "mapaB_CARRERA.txt"

    # Esta funcion busca y se acerca al objeto hasta estar en p
    def detect_scape(self):
        # Si la salida no se ha encontrado
        
        cam = picamera.PiCamera()

        #cam.resolution = (320, 240)
        cam.resolution = (640, 480)
        cam.framerate = 10 # less frame rate, more light BUT needs to go slowly (or stop)
        rawCapture = PiRGBArray(cam)
        
        # allow the camera to warmup
        time.sleep(0.2)

        # Se determinan puntos clave del mapa para ver los robots
        if self.mapa == 'A':
            first_table = [2000,1400]
            imgs_center = [2000,2800]
            center_table = [2000,2000]
            
        else:
            first_table = [800,1400]
            imgs_center = [800, 2800]
            center_table = [800,2000]
        
        # Se buscan las imagenes
        #self.go(first_table[0],first_table[1],150)
        #self.go(center_table[0],center_table[1],150)
        #self.align(imgs_center[0], imgs_center[1], np.deg2rad(1))
       
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
        while self.casilla_salida is None and x_face < (imgs_center[0] + 400):
            x_face += 200
            self.align(x_face, imgs_center[1], np.deg2rad(1))
            
            cam.capture(rawCapture, format="bgr")
            frame = rawCapture.array 
            
            self.detectar_casilla_salida(frame)

            rawCapture.truncate(0)
        
        
        cam.close()
        print('Salgo por la casilla', self.casilla_salida)
        time.sleep(1)
        
        # Una vez se ha encontrado la salida se sale
        #self.go(self.casilla_salida[0],self.casilla_salida[1])
        #self.go(self.casilla_salida[0],(self.casilla_salida[1] + 400))
    
    # Esta funcion busca y se acerca al objeto hasta estar en p
    def detect_scape_cv2(self):
        # Si la salida no se ha encontrado
        self.write_log()
        #time.sleep(2)
        # define a video capture object
        #vid = cv2.VideoCapture(0,cv2.CAP_DSHOW)
        vid = cv2.VideoCapture(0)
        
        # allow the camera to warmup
        time.sleep(2)

        # Se determinan puntos clave del mapa para ver los robots
        if self.mapa == 'A':
            first_table = [2000,1400]
            imgs_center = [2000,2800]
            center_table = [2000,2000]
            
        else:
            first_table = [800,1400]
            imgs_center = [800, 2800]
            center_table = [800,2000]
        
        #ESTO LO COMENTO PORQUE SOLO QUIERO COMPROBAR LA CAMARA
        # Se buscan las imagenes
        # self.go(first_table[0],first_table[1],150)
        #self.go(center_table[0],center_table[1],150)
        self.align(center_table[0], center_table[1], np.deg2rad(1))
        self.setSpeed(100,0)
        self.check_position_3_values(center_table[0], center_table[1], np.deg2rad(90), 10, np.Infinity, np.deg2rad(180))
        #self.align()
        self.align(imgs_center[0], imgs_center[1], np.deg2rad(1))
       
        _, frame = vid.read()
        

        self.detectar_casilla_salida(frame)

        # Si no lo ha encontardo yendo al centro del mapa se rota para buscar
        x_face = imgs_center[0]
        while self.casilla_salida is None and x_face > (imgs_center[0] - 400):
            x_face -= 200
            self.align(x_face, imgs_center[1], np.deg2rad(1))
            
            _, frame = vid.read()
            
            self.detectar_casilla_salida(frame)

            
        
        x_face = imgs_center[0]
        while self.casilla_salida is None and x_face < (imgs_center[0] + 400):
            x_face += 200
            self.align(x_face, imgs_center[1], np.deg2rad(1))
            
            _, frame = vid.read()

            self.detectar_casilla_salida(frame)

        print('Salgo por la casilla', self.casilla_salida)
        vid.release()

        cv2.destroyAllWindows()
        #time.sleep(1)

    def scape(self, vel):
        self.write_log()
        # Una vez se ha encontrado la salida se sale
       
        if self.mapa == 'A' and self.casilla_salida == [1400,2800]:
            self.salir_izquierda(vel)
        elif self.mapa == 'A' and self.casilla_salida == [2600,2800]:
            self.salir_derecha(vel)
        elif self.mapa == 'B' and self.casilla_salida == [200,2800]:
            self.salir_izquierda(vel)
        elif self.mapa == 'B' and self.casilla_salida == [1400,2800]:          
            self.salir_derecha(vel)
        self.write_log()

    def salir_izquierda(self,vel):
        print('salgo Izquierda')
        self.setSpeed(0,30)
        self.check_angle(np.deg2rad(180), np.deg2rad(1))
        # se lee la distancia a la pared 
        r = self.read_ultrasonyc() - 50
        v = vel
        w = np.rad2deg((float)(v/r))
        _, y, _ = self.readOdometry()
        self.setSpeed(v, -w)  # Primera semicircunferencia
        self.check_position_3_values(self.casilla_salida[0], y + r, np.deg2rad(90), np.Infinity, 10, np.deg2rad(180))
        self.setSpeed(0,-15)
        self.check_position_3_values(self.casilla_salida[0], 3000, np.deg2rad(90), np.Infinity,  np.Infinity, np.deg2rad(1))
        self.setSpeed(100,0)
        self.check_position_3_values(self.casilla_salida[0], 3000, np.deg2rad(90), np.Infinity, 10, np.deg2rad(180))

    def salir_derecha(self,vel):
        print('Salgo derecha')
        self.setSpeed(0,-30)
        self.check_angle(np.deg2rad(0), np.deg2rad(1))
        # se lee la distancia a la pared 
        r = self.read_ultrasonyc() - 50
        v = vel
        w = np.rad2deg((float)(v/r))
        _, y, _ = self.readOdometry()
        self.setSpeed(v, w)  # Primera semicircunferencia
        self.check_position_3_values(self.casilla_salida[0], y + r, np.deg2rad(90), np.Infinity, 10, np.deg2rad(180))
        self.setSpeed(0,15)
        self.check_position_3_values(self.casilla_salida[0], 3000, np.deg2rad(90), np.Infinity,  np.Infinity, np.deg2rad(1))
        self.setSpeed(100,0)
        self.check_position_3_values(self.casilla_salida[0], 3000, np.deg2rad(90), np.Infinity, 10, np.deg2rad(180))

    def relocate(self):
        self.setSpeed(0,15)
        self.check_angle(np.deg2rad(-180),np.deg2rad(1))
        x_axis = self.read_ultrasonyc()
        print('Leo de la X:', x_axis)
        self.setSpeed(0,30)
        self.check_angle(np.deg2rad(-90),np.deg2rad(1))
        y_axis = self.read_ultrasonyc()
        print('Leo de la Y:', y_axis)
        if y_axis < 800:
            if self.mapa == 'A':
                self.x.value = x_axis
            else:
                self.x.value = 2800 - x_axis
        else:
            if self.mapa == 'A':
                self.x.value = x_axis
            else:
                self.x.value = 2800 - x_axis
            self.y.value = y_axis 
        print('Se actualiza la odometria a:', x_axis, y_axis)       

        
    
    def read_ultrasonyc(self):
        """read_ultrasonyc toma 40 muestras del sensor del ultrasonidos y calcula la mediana
        para evitar datos espureos"""
        arr = []
        for i in range(20):
            try:
                arr.append(self.BP.get_sensor(self.BP.PORT_1)) 
            except brickpi3.SensorError as error:
                print(error) 
        dist = np.median(arr) * 10
        print("ULTRASONIDOS:", dist)
        return dist

    def detectar_casilla_salida(self, frame):
        """detectar_casilla_salida utiliza el sensor de lumniosidad para detectar 
        si se encuentra sobre la salida A o B (blanco y negro respectivamente)"""
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
    
    def seguimientoPared(self, dc):
        """seguimientoPared realiza uns eguimiento de pared derecha"""
        # PARÁMETROS
        k1 = 0.0006#15  # estabilizar
        k2 = -0.025  # amortiguar

        wmax = np.deg2rad(45)
        reached = False
        # velocidad lineal constante
        v = 100  # mm/s


        # Medida del sensor
        d = 0
        while d == 0: 
            try:
                d = self.BP.get_sensor(self.BP.PORT_3) * 10

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
            print(v,wc)
            self.setSpeed(v, np.rad2deg(wc))
            print('paso setSpeed')
            time.sleep(0.2)

            [_,y,_] = self.readOdometry()
            # Se comprueba que se ha alcanzado la salida
            if y > 3000:
                print('Se detiene porque llega')
                self.setSpeed(0,0)
                reached = True
            d_ant = d

            # Medida del sensor
            arr = []
            for i in range(5):
                try:
                    arr.append(self.BP.get_sensor(self.BP.PORT_3) * 10)
                    d = np.median(arr)
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































































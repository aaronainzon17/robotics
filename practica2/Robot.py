#!/usr/bin/python
# -*- coding: UTF-8 -*-
from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

#import brickpi3 # import the BrickPi3 drivers
import time     # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers
import sys
import numpy as np

# tambien se podria utilizar el paquete de threading
from multiprocessing import Process, Value, Array, Lock

class Robot:
    def __init__(self, init_position=[0.0, 0.0, 0.0]):
        """
        Initialize basic robot params. \

        Initialize Motors and Sensors according to the set up in your robot
        """

######## UNCOMMENT and FILL UP all you think is necessary (following the suggested scheme) ########

        # Robot construction parameters
        
        #Radio de la rueda
        self.R = Value('d',2.8)        #A lo mejor habria que ver si es 2.9      
        #Longitud entre ruedas
        self.L = Value('d',128.0)

        # self.v = Value('d',0.0)
        # self.w = Value('d',0.0)
        

        # self.x = Value('d',init_position[0])
        # self.y = Value('d',init_position[1])
        # self.th = Value('d',init_position[2])
        # self.finished = Value('b',1)

        ##################################################
        # Motors and sensors setup

        # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
        #self.BP = brickpi3.BrickPi3()
        self.BP = brickpi3.BrickPi3()

        # Configure sensors, for example a touch sensor.
        #self.BP.set_sensor_type(self.BP.PORT_1, self.BP.SENSOR_TYPE.TOUCH)
        self.BP.set_sensor_type(self.BP.PORT_1, self.BP.SENSOR_TYPE.TOUCH)

        # reset encoder B and C (or all the motors you are using)
        #self.BP.offset_motor_encoder(self.BP.PORT_B,
        #    self.BP.get_motor_encoder(self.BP.PORT_B))
        #self.BP.offset_motor_encoder(self.BP.PORT_C,
        #    self.BP.get_motor_encoder(self.BP.PORT_C))
        
        self.BP.offset_motor_encoder(self.BP.PORT_B,
           self.BP.get_motor_encoder(self.BP.PORT_B))
        self.BP.offset_motor_encoder(self.BP.PORT_C,
           self.BP.get_motor_encoder(self.BP.PORT_C))

        ##################################################
        # odometry shared memory values
        self.x = Value('d',0.0)
        self.y = Value('d',0.0)
        self.th = Value('d',0.0)
        self.finished = Value('b',1) # boolean to show if odometry updates are finished

        # if we want to block several instructions to be run together, we may want to use an explicit Lock
        self.lock_odometry = Lock()
        #self.lock_odometry.acquire()
        #print('hello world', i)
        #self.lock_odometry.release()

        # odometry update period --> UPDATE value!
        self.P = 0.1



    def setSpeed(self,v,w):
        """ To be filled - These is all dummy sample code """
        print("setting speed to %.2f %.2f" % (v, w))

        # compute the speed that should be set in each motor ...
        im0 = np.array([[1/self.R.value, self.L.value/2*self.R.value],[1/self.R.value, (-self.L.value)/(2*self.R.value)]])
        im1 = np.array([v,w])
        inverse_model = np.dot(im0,im1)
        wd = inverse_model[0]  
        wi = inverse_model[1]

        speedDPS_right = np.rad2deg(wd)
        speedDPS_left = np.rad2deg(wi)
        
        self.lock_odometry.acquire()
        #SC
        # self.v.value = v
        # self.w.value = w

        self.BP.set_motor_dps(self.BP.PORT_B, speedDPS_left)
        self.BP.set_motor_dps(self.BP.PORT_C, speedDPS_right)
        self.lock_odometry.release()


    def readSpeed(self):
        """ To be filled"""
        self.lock_odometry.acquire()
        #SC
        try:
            [encoder1, encoder2] = [self.BP.get_motor_encoder(self.BP.PORT_B), self.BP.get_motor_encoder(self.BP.PORT_C)]
        except IOError as error:
            print(error)
        grados_ruedas = np.array([[np.deg2rad(encoder1)] , [np.deg2rad(encoder2)]])
        trac = np.array([[self.R.value/2, self.R.value/2],[self.R.value/self.L.value,-self.R.value/self.L.value]])

        vel = np.dot(trac,grados_ruedas)

        try:
            self.BP.offset_motor_encoder(self.BP.PORT_B, self.BP.get_motor_encoder(self.BP.PORT_B)) # reset encoder B
            self.BP.offset_motor_encoder(self.BP.PORT_C, self.BP.get_motor_encoder(self.BP.PORT_C)) # reset encoder C
        except IOError as error:
            print(error)
        self.lock_odometry.release()
        return vel[0],vel[1]

    def readOdometry(self):
        """ Returns current value of odometry estimation """
        #return self.x.value, self.y.value, self.th.value
        self.lock_odometry.acquire()
        #SC
        x = self.x.value
        y = self.y.value
        th = self.th.value
        self.lock_odometry.release()
        return x,y,th

    def startOdometry(self):
        """ This starts a new process/thread that will be updating the odometry periodically """
        self.finished.value = False
        self.p = Process(target=self.updateOdometry, args=()) #additional_params?))
        self.p.start()
        print("PID: ", self.p.pid)
        self.log = open("log_odometry","w")

    # You may want to pass additional shared variables besides the odometry values and stop flag
    def updateOdometry(self): #, additional_params?):
        """ To be filled ...  """

        while not self.finished.value:
            # current processor time in a floating point value, in seconds
            tIni = time.clock()

            [realv,realw] = self.readSpeed()
                      
            realth = self.th.value + realw * self.P
            if realw == 0: 
                d_x = self.P * realv * np.cos(realth) # Duda de si es self.th.value 
                d_y = self.P *  realv * np.sin(realth)
            elif realw != 0:
                # El radio se calcula R = v/w 
                d_x = (realv/realw) * (np.sin(realth + realw + self.P) - np.sin(realth))
                d_y = (realv/realw) * (np.cos(realth + realw + self.P) - np.cos(realth))
            
            # # to "lock" a whole set of operations, we can use a "mutex"
            self.lock_odometry.acquire()
            self.x.value = d_x
            self.y.value = d_y
            self.th.value = realth 
            self.lock_odometry.release()

            # save LOG
            # Need to decide when to store a log with the updated odometry ...
            [x,y,th] = self.readOdometry()
            print(x + ',' + y + ',' + th + '\n', file=self.log)
            #self.log.write(x + ',' + y + ',' + th + '\n')
            ######## UPDATE UNTIL HERE with your code ########


            tEnd = time.clock()
            time.sleep(self.P - (tEnd-tIni))

        #print("Stopping odometry ... X= %d" %(self.x.value))
        #sys.stdout.write("Stopping odometry ... X=  %.2f, \
        #        Y=  %.2f, th=  %.2f \n" %(self.x.value, self.y.value, self.th.value))


    # Stop the odometry thread.
    def stopOdometry(self):
        self.finished.value = True
        
        self.BP.reset_all()


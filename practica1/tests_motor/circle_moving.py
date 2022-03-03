#!/usr/bin/env python

from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

import time     # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

#BP.set_sensor_type(BP.PORT_1, BP.SENSOR_TYPE.TOUCH) # Configure for a touch sensor. If an EV3 touch sensor is connected, it will be configured for EV3 touch, otherwise it'll configured for NXT touch.

BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B)) # Motor Izquierdo
BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C)) # Motor Derecho 

try:

    speedL = 20
    speedR = -20
    adder = 5
    finished = False
    running = False
    while not finished:
            
        if speedL >= 60:
            finished = 1 # this is the last iteration

        # Set the motor speed for two motors
        BP.set_motor_power(BP.PORT_B, speedL)
        BP.set_motor_power(BP.PORT_C, speedR)

        # delay for 2 seconds, to keep current speed for a little bit
        time.sleep(2.00)  
        #BP.set_motor_power(BP.PORT_B + BP.PORT_C, 0)
        
    
    # STOP the motors
    BP.set_motor_power(BP.PORT_B + BP.PORT_C, 0)

except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.

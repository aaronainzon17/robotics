#!/usr/bin/env python
#
# https://www.dexterindustries.com/BrickPi/
# https://github.com/DexterInd/BrickPi3
#
# Copyright (c) 2016 Dexter Industries
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
# For more information, see https://github.com/DexterInd/BrickPi3/blob/master/LICENSE.md
#
# This code is an example for reading an analog sensor connected to PORT_1 of the BrickPi3
# 
# Hardware: Connect an analog sensor (such as an NXT touch, light, or sound sensor) to sensor port 1 of the BrickPi3.
# 
# Results:  When you run this program, you will see the raw sensor value as well as the sensor voltage.

from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''
import numpy as np
import time     # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers


def setSpeed(v, w):
        """ Funcion que establece la velocidad lineal del robot a v y la velocidad
            angular del robot a w """

        #print("setting speed to %.2f %.2f" % (v, w))

        # Calculo de la velocidad a establecer para cada motor
        # segun las velocidades lineal y angular deseadas
        im0 = np.array([[1/26, 128/(2*26)],
                        [1/26, (-128)/(2*26)]])
        #print("im0", im0)
        im1 = np.array([v, np.deg2rad(w)])
        inverse_model = np.dot(im0, im1)
        #print("inverse_model", inverse_model)
        wd = inverse_model[0]
        wi = inverse_model[1]

        speedDPS_right = np.rad2deg(wd)
        speedDPS_left = np.rad2deg(wi)

        # Establece las velocidades de los motores con los valores calculados
        BP.set_motor_dps(BP.PORT_B, speedDPS_right)
        BP.set_motor_dps(BP.PORT_C, speedDPS_left)

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

BP.set_sensor_type(BP.PORT_4, BP.SENSOR_TYPE.CUSTOM, 
                            [(BP.SENSOR_CUSTOM.PIN1_ADC)]) # Configure for an analog on sensor port pin 1, and poll the analog line on pin 1.
 # reset encoder B and C (or all the motors you are using)
BP.offset_motor_encoder(BP.PORT_B,
                                BP.get_motor_encoder(BP.PORT_B))  # RUEDA DERECHA
BP.offset_motor_encoder(BP.PORT_C,
                                     BP.get_motor_encoder(BP.PORT_C))  # RUEDA IZQUIERDA

#setSpeed(0,60)
try:
    GYRO_DEFAULT = 2449.47
    GYRO2DEG = 0.24
    acum = []
    i = 0
    while i < 1500:
        # read the sensor value
        # BP.get_sensor retrieves a sensor value.
        # BP.PORT_1 specifies that we are looking for the value of sensor port 1.
        # BP.get_sensor returns a list of 4 values.
        #     The first is the pin 1 analog line value (what we want to display).
        #     The second is the pin 6 analog line value.
        #     The third is the pin 5 digital value.
        #     The fourth is the pin 6 digital value.
        
        try:
            gyro_data = BP.get_sensor(BP.PORT_4)[0]
            acum.append(gyro_data)
            i+=1
            #gyro_speed = (GYRO_DEFAULT - gyro_data)* GYRO2DEG
            print(gyro_data)
            #value = BP.get_sensor(BP.PORT_4)[0] - avg_gyroscppe # read the sensor port value
            #print("Raw value: %4d   Voltage: %5.3fv" % (value, (value / (4095.0 / BP.get_voltage_5v())))) # print the raw value, and calculate and print the voltage as well
        except brickpi3.SensorError as error:
            print(error)
        
        time.sleep(0.02)  # delay for 0.02 seconds (20ms) to reduce the Raspberry Pi CPU load../g   
    print('la mediana',np.median(acum))
except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.
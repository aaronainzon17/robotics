#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
from tokenize import Double
import numpy as np
import time
import math
from Robot import Robot


# La funcion dos_puntos_time es una primera version de la trayectoria del 
# las dos semicirunferencias unidas caluclada con tiempos con velocidad 
# lineal 200 y angular 90
def dos_puntos_time(robot, a, d, dist):
    r2 = math.sqrt(dist**2 + (d-a)**2)
    th = np.rad2deg(math.acos((dist**2 + r2**2 - (d-a)**2)/(2*dist*r2)))

    v = 200
    w = 90
    w1 = np.rad2deg((float)(v/a))
    w2 = np.rad2deg((float)(v/d))

    robot.setSpeed(0, w)    # gira 90 grados a la izquierda
    time.sleep(90/w)
    robot.setSpeed(v, -w1)  # cuarto de circunferencia a la derecha
    time.sleep(90/w1)
    robot.setSpeed(0, w)    # gira th grados a la izquierda
    time.sleep(th/w)
    robot.setSpeed(v, 0)    # avanza r2 mm en linea recta
    time.sleep(r2/v)
    robot.setSpeed(0, -w)   # gira th grados a la derecha
    time.sleep(th/w)
    robot.setSpeed(v, -w2)  # media circunferencia a la derecha
    time.sleep(180/w2)
    robot.setSpeed(0, -w)   # gira th grados a la derecha
    time.sleep(th/w)
    robot.setSpeed(v, 0)    # avanza r2 mm en linea recta
    time.sleep(r2/v)
    robot.setSpeed(0, w)    # gira th grados a la izquierda
    time.sleep(th/w)
    robot.setSpeed(v, -w1)  # cuarto de circunferencia a la derecha
    time.sleep(90/w1)
    robot.setSpeed(0, 0)



# La funcion ocho_time es una primera version de la trayectoria del 
# ocho caluclada con tiempos con velocidad lineal 200
def ocho_time(robot, d):
    v = 200
    w = np.rad2deg((float)(v/d))

    robot.setSpeed(0, -90)  # Primer giro
    time.sleep(1)
    robot.setSpeed(v, w)    # Semicirmunferencia incial
    time.sleep(180/w)
    robot.setSpeed(v, -w)   # CIrculo mas lejano
    time.sleep(360/w)
    robot.setSpeed(v, w)    # Semicirunferencia final
    time.sleep(180/w)
    robot.setSpeed(0, 0)    # Parar robot


# La funcion rectangulo_time es una primera version de la trayectoria del 
# rectangulo caluclada con tiempos con velocidad lineal 200 y angular 60
def rectangulo_time(robot):
    robot.setSpeed(200, 0)
    time.sleep(4)
    robot.setSpeed(0, 60)
    time.sleep(1.5)
    robot.setSpeed(200, 0)
    time.sleep(2)
    robot.setSpeed(0, 60)
    time.sleep(1.5)
    robot.setSpeed(200, 0)
    time.sleep(4)
    robot.setSpeed(0, 60)
    time.sleep(1.5)
    robot.setSpeed(200, 0)
    time.sleep(2)
    robot.setSpeed(0, 60)
    time.sleep(1.5)


# FUncion de normalizacion del angulo entre -pi, pi
def normalizar(th):
    if th > math.pi:
        th -= (2 * math.pi)
    elif th < -math.pi:
        th += (2 * math.pi)
    return th

# check_position es la funcion de control de localizacion 
# En ella se comprueba la posicion real del robot leida de los 
# motores y se comprueba si se encuentra en la posicion deseada 
# permitiendo un cierto error.
def check_position(robot, x, y, th, x_err, y_err, angular_err):
    # Se lee incialmente la posicion del robot
    [x_now, y_now, th_now] = robot.readOdometry()
    reached = False
    
    while not reached:
        print("-------------------------------------------")
        print("quiero llegar a ", x, " ", y, " ", th)
        print("estoy en ", x_now, " ", y_now, " ", th_now)
        print("-------------------------------------------")
        
        # Se calcula el angulo
        error_ang = abs(th-th_now)
        if error_ang > math.pi: 
            error_ang = (2*math.pi) - error_ang
        
        # Se comprueba que la trayectoria se esta relaizando dentro del error permitido
        if (abs(x-x_now) <= x_err) and (abs(y-y_now) <= y_err) and (error_ang <= angular_err):
            reached = True
            print("Se ha alcanzado el punto:[",
                  x_now, ",", y_now, ",", th_now, "]")
        else:
            [x_now, y_now, th_now] = robot.readOdometry()
            print("La posicion actual es:", x_now, y_now)


# La funcion rectangulo realiza la trayectoria del rectangulo basandose en 
# la odometria para detener al robot y comenzar con el siguiente movimiento
def rectangulo(robot, base, altura, vel, vel_giro):
    v = vel
    w = vel_giro
    robot.setSpeed(v, 0) # Primera recta (base del rectangulo)
    check_position(robot, base, 0, 0, 3, 3, np.deg2rad(2))

    robot.setSpeed(0, w) # Giro 90 deg
    check_position(robot, base, 0, normalizar(
        np.deg2rad(90)), np.Infinity, np.Infinity, np.deg2rad(4))

    robot.setSpeed(v, 0) # Segunda recta (altura del rectangulo)
    check_position(robot, base, altura, normalizar(
        np.deg2rad(90)), 20, 20, np.deg2rad(4))

    robot.setSpeed(0, w) # Giro 90 deg
    check_position(robot, base, altura, normalizar(
        np.deg2rad(180)), np.Infinity, np.Infinity, np.deg2rad(2))

    robot.setSpeed(v, 0) # Tercera recta (base del rectangulo)
    check_position(robot, 0, altura, normalizar(
        np.deg2rad(180)), 5, 40, np.deg2rad(5)) 

    robot.setSpeed(0, w) # Giro 90 deg
    check_position(robot, 0, altura, normalizar(
        np.deg2rad(270)), np.Infinity, np.Infinity, np.deg2rad(2))

    robot.setSpeed(v, 0) # Recta final (altura del rectangulo)
    check_position(robot, 0, 0, normalizar(
        np.deg2rad(270)), 40, 5, np.deg2rad(5))

    robot.setSpeed(0, w) # Ultimo giro de 90 deg
    check_position(robot, 0, 0, normalizar(
        np.deg2rad(0)), np.Infinity, np.Infinity, np.deg2rad(2))

    robot.setSpeed(0, 0) # Detener el robot 

# La funcion ocho realiza la trayectoria del ocho basandose en 
# la odometria para detener al robot y comenzar con el siguiente movimiento
def ocho(robot, d, vel):
    v = vel
    w = np.rad2deg((float)(v/d))

    robot.setSpeed(0, -45) # Giro de 90 deg a la derecha 
    check_position(robot, 0, 0, normalizar(np.deg2rad(-90)),np.Infinity, np.Infinity, np.deg2rad(2))

    robot.setSpeed(v, w) # Primera semicircunferencia
    check_position(robot, 2*d, 0, normalizar(np.deg2rad(90)),10, 5, np.deg2rad(10))

    robot.setSpeed(v, -w) # Segunda semicircunferencia
    check_position(robot, 4*d, 0, normalizar(np.deg2rad(-90)),20, 5, np.deg2rad(10))

    robot.setSpeed(v, -w) # Tercera semicircunferencia
    check_position(robot, 2*d, 0, normalizar(np.deg2rad(90)),30, 5, np.deg2rad(10))

    robot.setSpeed(v, w) # Ultima semicircunferencia
    check_position(robot, 0, 0, normalizar(np.deg2rad(-90)),5, 5, np.deg2rad(5))

    robot.setSpeed(0, 0) # Parar el robot


# La funcion dos_puntos realiza la trayectoria dedos semicirculos unidos basandose
#  en la odometria para detener al robot y comenzar con el siguiente movimiento
def dos_puntos(robot, a, d, dist, vel):
    r2 = math.sqrt(dist**2 + (d-a)**2)
    th = np.rad2deg(math.acos((dist**2 + r2**2 - (d-a)**2)/(2*dist*r2)))

    v = vel
    v1 = vel
    w = 45
    w1 = np.rad2deg((float)(v1/a))
    w2 = np.rad2deg((float)(v1/d))

    robot.setSpeed(0, w)    # gira 90 grados a la izquierda
    check_position(robot, 0, 0, normalizar(np.deg2rad(90)),
                   2, 2, np.deg2rad(1))

    robot.setSpeed(v1, -w1)  # cuarto de circunferencia a la derecha
    check_position(robot, a, a, normalizar(np.deg2rad(0)),
                   10, 10, np.deg2rad(10))

    robot.setSpeed(0, w)    # gira th grados a la izquierda
    check_position(robot, a, a, normalizar(np.deg2rad(th)),
                   12, 12, np.deg2rad(4))

    robot.setSpeed(v, 0)    # avanza r2 mm en linea recta
    check_position(robot, a+dist, d, normalizar(np.deg2rad(th)),
                   13, 13, np.deg2rad(2))

    robot.setSpeed(0, -w)   # gira th grados a la derecha
    check_position(robot, a+dist, d, normalizar(np.deg2rad(0)),
                   16, 16, np.deg2rad(4))

    robot.setSpeed(v1, -w2)  # media circunferencia a la derecha
    check_position(robot, a+dist, -d, normalizar(np.deg2rad(180)),
                   30, 30, np.deg2rad(10))

    robot.setSpeed(0, -w)   # gira th grados a la derecha
    check_position(robot, a+dist, -d, normalizar(np.deg2rad(180-th)),
                   33, 33, np.deg2rad(4))

    robot.setSpeed(v, 0)    # avanza r2 mm en linea recta
    check_position(robot, a, -a, normalizar(np.deg2rad(180-th)),
                   36, 36, np.deg2rad(4))

    robot.setSpeed(0, w)    # gira th grados a la izquierda
    check_position(robot, a, -a, normalizar(np.deg2rad(180)),
                   np.Infinity, np.Infinity, np.deg2rad(4))

    robot.setSpeed(v1, -w1)  # cuarto de circunferencia a la derecha
    check_position(robot, 0, 0, normalizar(np.deg2rad(90)),
                   40, 40, np.deg2rad(10))
    robot.setSpeed(0, 0)


def main(args):
    try:
        if args.radioD < 0:
            print('d must be a positive value')
            exit(1)

        # Inicializar odometria en [0,0,0]
        robot = Robot()

        print("X value at the beginning from main X= %.2f" % (robot.x.value))

        # 1. Se incia la odometria u el proceso update odometry
        robot.startOdometry()

        # 2. perform trajectory

        print("Start : %s" % time.ctime())

        # Trayectoria de Rectangulo
        rectangulo(robot, 800, 400, 150,45)
        
        #Trayectoria de Ocho
        #ocho(robot, 400, 150)
        
        #Trayectoria de Dos circulos
        #dos_puntos(robot,200,400,600,150)

        print("End : %s" % time.ctime())

        robot.lock_odometry.acquire()
        print("Odom values at main at the END: %.2f, %.2f, %.2f " %
              (robot.x.value, robot.y.value, robot.th.value))
        robot.lock_odometry.release()

        # 3. wrap up and close stuff ...
        # This currently unconfigure the sensors, disable the motors,
        # and restore the LED to the control of the BrickPi3 firmware.
        robot.stopOdometry()

    except KeyboardInterrupt:
        # except the program gets interrupted by Ctrl+C on the keyboard.
        # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        robot.stopOdometry()


if __name__ == "__main__":

    # get and parse arguments passed to main
    # Add as many args as you need ...
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--radioD", help="Radio to perform the 8-trajectory (mm)",
                        type=float, default=40.0)
    args = parser.parse_args()

    main(args)

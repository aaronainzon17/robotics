#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
from Robot import Robot

def main(args):
    try:
        if args.radioD < 0:
            print('d must be a positive value')
            exit(1)

        # Se instancia el robot
        robot = Robot() 
        # Se inicia la odometria
        robot.startOdometry()
        # Se inicia la busqueda de la pelota 
        robot.trackObject(colorRangeMin=[0,0,0], colorRangeMax=[255,255,255])
        # Se detiene la odometria
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

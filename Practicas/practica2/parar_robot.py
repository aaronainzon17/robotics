#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
from tokenize import Double
from Robot import Robot

def main(args):
    try:
        robot = Robot()

        robot.startOdometry()

        robot.setSpeed(0, 0)

        robot.stopOdometry()

    except KeyboardInterrupt:

        robot.stopOdometry()


if __name__ == "__main__":

    # get and parse arguments passed to main
    # Add as many args as you need ...
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--radioD", help="Radio to perform the 8-trajectory (mm)",
                        type=float, default=40.0)
    args = parser.parse_args()

    main(args)

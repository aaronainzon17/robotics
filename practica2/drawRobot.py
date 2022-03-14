#!/usr/bin/python

import matplotlib.pyplot as plt
import sys
import numpy as np
import homYloc as h
import simubot as s
import dibrobot as d

if len(sys.argv) < 1:
    print("Usage: drawRobot.py <file_name>") 
    exit(1)

file_name = str(sys.argv[1])
f = open(file_name, "r")

for x in f:
    coords = x.split(",")
    b = coords[2].split('\n')
    a = np.array([float(coords[0]),float(coords[1]),float(b[0])])
    d.dibrobot(a,'purple','s')
plt.show()
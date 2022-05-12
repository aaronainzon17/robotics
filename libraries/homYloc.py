#Author: Aaron Ibañez Espes
#Date: 23/02/2022

import numpy as np

def hom(x):
    x1 = x[0]; y1 = x[1]; th = x[2]

    T = np.array([
        [np.cos(th), - np.sin(th), x1],
        [np.sin(th), np.cos(th), y1],
        [0, 0, 1]])

    return T

def loc(T):
    nx = T[0][0]; ny = T[1][0]
    px = T[0][2]; py = T[1][2]
    x = np.array([px, py,np.arctan2(ny,nx)])

    return x
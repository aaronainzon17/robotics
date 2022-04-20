import numpy as np
import math 
import matplotlib.pyplot as plt
def lienar_w(dth):
    w = dth*(80/math.pi)
    if w < 0:
        w -= 10
    else:
        w+= 10
    return w

#Funcion que normaliza el angulo entre -pi, pi
def norm_pi(th):
        if th > math.pi:
            th = th - 2 * math.pi
        elif th < -math.pi:
            th = th + 2 * math.pi
        return th

[x,y,th] = [0,0,np.deg2rad(90)]

[x_goal,y_goal,th_goal] = [2,10,np.deg2rad(180)]

dx = x_goal - x
dy = y_goal - y
dth = norm_pi(np.arctan2(dy, dx) - th)
print(dth, np.rad2deg(dth))
print(lienar_w(dth))
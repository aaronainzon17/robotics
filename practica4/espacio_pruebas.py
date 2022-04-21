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

def align(x_goal, y_goal, error_ang=np.deg2rad(3)):
            aligned = False 
            
            #while not aligned:
            [x_now, y_now, th_now] = [200,200,np.deg2rad(90)]
            d_x = x_goal - x_now
            d_y = y_goal - y_now
            d_th = norm_pi(np.arctan2(d_y, d_x) - th_now)
            print(d_th)
            print('El error', error_ang)
            if abs(d_th) < error_ang:
                print('Esta alienado')
                aligned = True
            else:
                print('Tengo que girar', d_th)

align(600,600,np.deg2rad(5))
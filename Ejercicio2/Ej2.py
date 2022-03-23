from operator import index
from tkinter import W
import numpy as np
import math 
import tkinter
import matplotlib.pyplot as plt

pi = 3.141592653589793
tau = 6.283185307179586476925287


# Dibuja robot en location_eje con color (c) y tamano (p/g)
def dibrobot(loc_eje,c,tamano):
  if tamano=='p':
    largo=0.1
    corto=0.05
    descentre=0.01
  else:
    largo=0.5
    corto=0.25
    descentre=0.05

  trasera_dcha=np.array([-largo,-corto,1])
  trasera_izda=np.array([-largo,corto,1])
  delantera_dcha=np.array([largo,-corto,1])
  delantera_izda=np.array([largo,corto,1])
  frontal_robot=np.array([largo,0,1])
  tita=loc_eje[2]
  Hwe=np.array([[np.cos(tita), -np.sin(tita), loc_eje[0]],
             [np.sin(tita), np.cos(tita), loc_eje[1]],
              [0,        0 ,        1]])
  Hec=np.array([[1,0,descentre],
              [0,1,0],
              [0,0,1]])
  extremos=np.array([trasera_izda, delantera_izda, delantera_dcha, trasera_dcha, trasera_izda, frontal_robot, trasera_dcha])
  robot=np.dot(Hwe,np.dot(Hec,np.transpose(extremos)))
  
  plt.plot(robot[0,:], robot[1,:], c)
  plt.plot(0,10)
  plt.plot(0,0)
  plt.plot(5,0)
  
  plt.show()

def norm_pi(num):
    return (num + pi)%tau - pi

def simubot(vc,xWR,T):
  if vc[1]==0:   # w=0
      xRk=np.array([vc[0]*T, 0, 0])
  else:
      R=vc[0]/vc[1]
      dtitak=vc[1]*T
      titak=norm_pi(dtitak);
      xRk=np.array([R*np.sin(titak), R*(1-np.cos(titak)), titak])  

  xWRp=loc(np.dot(hom(xWR),hom(xRk)))   # nueva localizaci�n xWR
  return xWRp

#hom(X) dada una lista de [x,y,tita] se obtiene la matriz homogenea
# X debe ser una lista de la forma [x,y,tita]
def hom(X):

    #cos y sen funcionan con radianes por eso se transforman los grados en radianes
    matrizHomogenea = np.array([[math.cos(X[2]) ,-math.sin(X[2]),X[0]],[math.sin(X[2]) ,math.cos(X[2]),X[1]],[0,0,1]])
    return matrizHomogenea

#La funcion recibe una matriz y devuelve la lista [x,y,tita]
def loc(T):
    #coor=np.array([T[0][2],T[1][2],math.degrees(math.atan2(T[1][0],T[0][0]))])
    coor=np.array([T[0][2],T[1][2],math.atan2(T[1][0],T[0][0])])
    return coor



"""
do
    gxr = loc(hom(gxw)*hom(wxr))
    gpr_plus = polares(gxr) #polares sera una funcion de python o hecha por nosotros
    [vr,wr] = K * gpr_plus
    wxr_plus = simutobot(wxr,vr,wr)
    wxr = wxr_plus
while   !ultimo_objetivo
"""

#Si te dan como se ve algo desde el mundo, te devuelve como se ve el mundo desde algo
def invertir(wxg):
    gwx = wxg * -1
    return gwx

#def cart2pol(x, y, th):
def cart2pol(x, y, th):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    alpha = phi - th
    return np.array([rho, phi,alpha])

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)

def Ej2():
    indexObjectivoActual = 0
    recorrido = np.array([[2,3,np.deg2rad(90)],[6,4,np.deg2rad(45)],[10,5,np.deg2rad(-45)],[7,-3,np.deg2rad(180)],[2,3,np.deg2rad(90)]])
    objetivoActual = recorrido[0]
    wxr = np.array([0,0,0]) #esta es la inicial
    gxw = invertir(objetivoActual)
    objetivoAlcanzado = False
    K = np.array([[0.8,0,0],[0,0.9,0.8]])
    

    while not objetivoAlcanzado:
        gxr = loc(hom(gxw)*hom(wxr))
        gpr_plus = cart2pol(gxr[0],gxr[1],gxr[2])
        V = K * gpr_plus;
        vr = V[0][0]
        wr = V[1][0]
        wxr_plus = simubot(wxr,vr,wr)
        wxr = wxr_plus
        if wxr == objetivoActual:
            dibrobot(wxr,'c','g')
            indexObjectivoActual+=1
            objetivoActual = recorrido[indexObjectivoActual]
            if(indexObjectivoActual == 5):  #Ha acabado la trayectoria
                objetivoAlcanzado = True




Ej2()
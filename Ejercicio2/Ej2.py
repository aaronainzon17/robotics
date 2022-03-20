from tkinter import W
import numpy as np
import math 
import tkinter
import matplotlib.pyplot as plt

"""
do
    gxr = loc(hom(gxw)*hom(wxr))
    gpr_plus = polares(gxr) #polares sera una funcion de python o hecha por nosotros
    [vr,wr] = K * gpr_plus
    wxr_plus = simutobot(wxr,vr,wr)
    wxr = wxr_plus
while   !ultimo_objetivo
"""

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

    while not objetivoAlcanzado:
        gxr = loc(hom(gxw)*hom(wxr))
        gpr_plus = cart2pol(gxr[0],gxr[1],gxr[2])




    


# print(np.deg2rad(90))
# print(np.deg2rad(45))
# print(np.deg2rad(-45))
# print(np.deg2rad(180))
# print(np.deg2rad(90))
Ej2()
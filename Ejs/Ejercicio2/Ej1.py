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

def mostrarCoor(coor):
    print("x= ",coor[0], " y= ",coor[1], " tita= ", math.degrees(coor[2]))

origenCoor = np.array([0,0,0])
wxp = np.array([2.5,10,0])
rxp = np.array([5.2,-3,math.radians(-125)])
rxc = np.array([4.17,0.73,math.radians(-35)])
r1xr2 = rxp

#Apartado 1
#Calcular la localización del robot en W, si la localización relativa
#entre puerta y robot es Rx P=(5.2,-3,-125gr).
#Recibe como parametros la localizacion de la puerta respecto del mundo y la localizacion de la puerta respecto del robot
def apartado1(wxp,rxp):
    #Ecuacion: wtp * (rtp)-1 = wtr
    wtr = np.matmul(hom(wxp), np.linalg.inv(hom(rxp)))
    #wtr = np.matmul(np.linalg.inv(hom(rxp)), hom(wxp))
    wxr = loc(wtr)
    return wxr

wxr = apartado1(wxp,rxp)

print()
#devuelve wxc
def apartado2(wxr,rxc):
    wtc = np.matmul(hom(wxr),hom(rxc))
    wxc = loc(wtc)
    return wxc

wxc = apartado2(wxr,rxc)

#devuelve pxc
def apartado3(rxp,rxc):
    ptc = np.matmul(np.linalg.inv(hom(rxp)),hom(rxc))
    pxc = loc(ptc)
    return pxc

pxc = apartado3(rxp,rxc)

print()
def apartado4(rxp, T , wxr):
    x = rxp[0]
    y = rxp[1]
    Ra = (pow(x,2) + pow(y,2))/(2*y)
    th2 = math.atan2(2*x*y,pow(x,2) - pow(y,2))                     #En radianes
    r1xr2 = np.array([x,y,th2])
    w = th2 / T ;
    v = Ra*w
    wtr2 = np.matmul(hom(wxr),hom(r1xr2))
    wxr2 = loc(wtr2)
    return [v,w,r1xr2,wxr2,Ra] 

resultsApartado4 = apartado4(rxp,8,wxr)

xWRp = simubot([resultsApartado4[0],resultsApartado4[1]],wxr,8)

print()
def apartado5(w,T,R,wxr):
    thk = w*T
    xk = R*math.sin(thk)
    yk = R*(1-math.cos(thk))
    r1xr3 = np.array([xk,yk,thk])
    wtr3 = np.matmul(hom(wxr),hom(r1xr3))
    wxr3 = loc(wtr3)
    return [r1xr3,wxr3]

resultsApartado5 = apartado5(resultsApartado4[1],4,resultsApartado4[4],wxr)
xWRp5 = simubot([resultsApartado4[0],resultsApartado4[1]],wxr,4)


#Mostrar todos los resultados aqui
print("--------------------------------")
print("Resultados apartado 1")
print("Localizacion del robot en el mundo")
mostrarCoor(wxr)
dibrobot(wxr,'c','g')
print("--------------------------------")
print()
print("--------------------------------")
print("Resultados apartado 2")
print("Localizacion del cuadro en el mundo")
mostrarCoor(wxc)
dibrobot(wxc,'c','g')
print("--------------------------------")
print()
print("--------------------------------")
print("Resultados apartado 3")
print("Localizacion del cuadro respecto de la puerta")
mostrarCoor(pxc)
print("--------------------------------")
print()
print("--------------------------------")
print("Resultados apartado 4")
print("La velocidad lineal es ", resultsApartado4[0], " la velocidad angular es ", resultsApartado4[1])
print("La localizacion final del robot respecto de la inical es")
mostrarCoor(resultsApartado4[2])
print("La localizacion final del robot en el mundo es")
mostrarCoor(resultsApartado4[3])
print("--------------------------------")
print()
print("--------------------------------")
print("Resultados apartado 5")
print("La localizacion final del robot respecto de la inical es")
mostrarCoor(resultsApartado5[0])
print("La localizacion final del robot en el mundo es")
mostrarCoor(resultsApartado5[1])
print("--------------------------------")

#Se simula el robot para los ultiumos apartados
dibrobot(resultsApartado4[3],'c','g')
dibrobot(resultsApartado5[1],'c','g')
# -*- coding: utf-8 -*-
import numpy as np

...

# Simula movimiento del robot con vc=[v,w] en T seg. desde xWR
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

...
print("Hola")
simubot([45.02242845222209,-7.495409842212333],,8)
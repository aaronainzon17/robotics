from re import X
import numpy as np
import math 
import matplotlib.pyplot as plt

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

#Funcion que normaliza el angulo entre -pi, pi
def norm_pi(th):
        if th > math.pi:
            th = th - 2 * math.pi
        elif th < -math.pi:
            th = th + 2 * math.pi
        return th
        
# Simula movimiento del robot con vc=[v,w] en T seg. desde xWR
def simubot(vc,xWR,T):
  if vc[1]==0:   # w=0
      xRk=np.array([vc[0]*T, 0, 0])
  else:
      R=vc[0]/vc[1]
      dtitak=vc[1]*T
      titak=norm_pi(dtitak)
      xRk=np.array([R*np.sin(titak), R*(1-np.cos(titak)), titak])  

  xWRp=loc(np.dot(hom(xWR),hom(xRk)))   # nueva localizaci�n xWR

  return xWRp

# A partir de un punto crea la matriz en coordenadas homogeneas 
def hom(x):
    x1 = x[0]; y1 = x[1]; th = x[2]

    T = np.array([
        [np.cos(th), - np.sin(th), x1],
        [np.sin(th), np.cos(th), y1],
        [0, 0, 1]])

    return T

# A partir de una matriz calcula el punto
def loc(T):
    nx = T[0][0]; ny = T[1][0]
    px = T[0][2]; py = T[1][2]
    x = np.array([px, py,norm_pi(np.arctan2(ny,nx))])

    return x

# Transforma de coordenadas cartesianas a polares
def cart2pol(x, y, th):
    rho = np.sqrt(np.power(x,2) + np.power(y,2))
    beta = norm_pi(np.arctan2(y, x) + math.pi)
    alpha = beta - th
    return np.array([[rho], [alpha], [beta]])

# Transforma de coordenadas cartesianas a polares
def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)

def Ej2():
    indexObjectivoActual = 0
    # Vector con los puntos por los que se debe pasar 
    recorrido = np.array([[2,3,np.deg2rad(90)],[6,4,np.deg2rad(45)],[10,5,np.deg2rad(-45)],[7,-3,np.deg2rad(180)],[2,3,np.deg2rad(90)]])
    objetivoActual = recorrido[0]
    wxr = np.array([0,0,np.deg2rad(0)]) # Posicion inical del robot con respecto al mundo
    
    objetivoAlcanzado = False
    k = np.array([[0.1,0,0],[0,0.7,0.5]])
    
    # Variables auxiliares para el plot de la grafica
    i = 0
    vel_axis = [0]
    w_vel_axis = [0]
    x_axis = 0
    # Bulce en el que se recorren los puntos marcados 
    while not objetivoAlcanzado:
        
        # Se calcula la localizacion del robot con respecto al goal (punto objetivo)
        gTr = np.dot(np.linalg.inv(hom(objetivoActual)),hom(wxr))
        gxr = loc(gTr)
        
        # se transforma a coordenadas polares
        gpr_plus = cart2pol(gxr[0],gxr[1],gxr[2])
        
        # se calculan la velocidad lineal y angular 
        vel = np.dot(k,gpr_plus)
        vr = vel[0][0]
        wr = vel[1][0]
        
        # Aux para el plot de las grafiacas de v y w
        vel_axis.append(vr)
        w_vel_axis.append(wr)

        # Comprobacion de que no se supera en ningun caso 3 m/s o 3 rad/s
        if vr > 3 or wr > 3: 
            print(vr , wr)

        # Se simula el movimiento 
        wxr_plus = simubot(np.array([vr,wr]),wxr,i)
        wxr = wxr_plus
        dibrobot(wxr,'c','p')
        
        # Se comprueba que se ha alcanzado el punto con un error minimo (debido a que son floats y nunca se alcanza 0)
        if gxr[0] < 0.05 and gxr[0] > -0.05 and gxr[1] < 0.05 and gxr[1] > -0.05 and gxr[2] < 0.05 and gxr[2] > -0.05:
        
            indexObjectivoActual += 1
            if(indexObjectivoActual == 5):  #Ha acabado la trayectoria
                objetivoAlcanzado = True
            else:
                print('Trayectoria', indexObjectivoActual)
                objetivoActual = recorrido[indexObjectivoActual]
        i += 0.00001
        x_axis += 1         
    
    # Muestra el recorrido 
    plt.show()

    # Muestra la grafica de velocidad 
    plt.plot(range(0, x_axis+1),w_vel_axis, color='red')
    plt.plot(range(0, x_axis+1),vel_axis, color='blue')
    plt.show()




Ej2()
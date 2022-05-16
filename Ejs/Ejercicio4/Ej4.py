# Autores (Nombre Apellido - NIA):
# Sergio Gabete - 774631
# Pablo Gancedo - 736839
# Belén Gimeno - 756425
# Aarón Ibáñez - 779088

from pydoc import source_synopsis
import numpy as np
import math
import matplotlib.pyplot as plt

# Dibuja robot en location_eje con color (c) y tamano (p/g)


def dibrobot(loc_eje, c, tamano):
    if tamano == 'p':
        largo = 0.1
        corto = 0.05
        descentre = 0.01
    else:
        largo = 0.5
        corto = 0.25
        descentre = 0.05

    trasera_dcha = np.array([-largo, -corto, 1])
    trasera_izda = np.array([-largo, corto, 1])
    delantera_dcha = np.array([largo, -corto, 1])
    delantera_izda = np.array([largo, corto, 1])
    frontal_robot = np.array([largo, 0, 1])
    tita = loc_eje[2]
    Hwe = np.array([[np.cos(tita), -np.sin(tita), loc_eje[0]],
                    [np.sin(tita), np.cos(tita), loc_eje[1]],
                    [0,        0,        1]])
    Hec = np.array([[1, 0, descentre],
                    [0, 1, 0],
                    [0, 0, 1]])
    extremos = np.array([trasera_izda, delantera_izda, delantera_dcha,
                         trasera_dcha, trasera_izda, frontal_robot, trasera_dcha])
    robot = np.dot(Hwe, np.dot(Hec, np.transpose(extremos)))

    plt.plot(robot[0, :], robot[1, :], c)

# Funcion que normaliza el angulo entre -pi, pi


def norm_pi(th):
    if th > math.pi:
        th = th - 2 * math.pi
    elif th < -math.pi:
        th = th + 2 * math.pi
    return th

# Simula movimiento del robot con vc=[v,w] en T seg. desde xWR


def simubot(vc, xWR, T):
    if vc[1] == 0:   # w=0
        xRk = np.array([vc[0]*T, 0, 0])
    else:
        R = vc[0]/vc[1]
        dtitak = vc[1]*T
        titak = norm_pi(dtitak)
        xRk = np.array([R*np.sin(titak), R*(1-np.cos(titak)), titak])

    xWRp = loc(np.dot(hom(xWR), hom(xRk)))   # nueva localización xWR

    return xWRp

# A partir de un punto crea la matriz en coordenadas homogeneas


def hom(x):
    x1 = x[0]
    y1 = x[1]
    th = x[2]

    T = np.array([
        [np.cos(th), - np.sin(th), x1],
        [np.sin(th), np.cos(th), y1],
        [0, 0, 1]])

    return T

# A partir de una matriz calcula el punto


def loc(T):
    nx = T[0][0]
    ny = T[1][0]
    px = T[0][2]
    py = T[1][2]
    x = np.array([px, py, norm_pi(np.arctan2(ny, nx))])

    return x


def ej4(pared, dc, pos_ini, ang_ini):
    if pared == "d":
        # PARÁMETROS Pared derecha
        # valores iniciales: k1 = 0.15 y k2 = -0.5
        k1 = 0.01  # estabilizar
        k2 = -5  # amortiguar

        # Dibuja las paredes
        x_seguir = [0, 35]
        y_seguir = [0, 0]
        plt.plot(x_seguir, y_seguir)

        x_meta = [35, 35]
        y_meta = [0, 1.5]
        plt.plot(x_meta, y_meta)
    elif pared == "i":
        # PARÁMETROS Pared izquierda
        # k1 y k2 diferentes
        k1 = -0.01  # estabilizar
        k2 = 5  # amortiguar

        # Dibuja las paredes
        x_seguir = [0, -35]
        y_seguir = [0, 0]
        plt.plot(x_seguir, y_seguir)

        x_meta = [-35, -35]
        y_meta = [0, 1.5]
        plt.plot(x_meta, y_meta)
    else:
        print("¿Cómo?")
        return

    wmax = np.deg2rad(45)
    reached = False
    # velocidad lineal constante
    v = 0.02  # m/s

    wxr = np.array([0.0, pos_ini, np.deg2rad(ang_ini)])
    dibrobot(wxr, 'r', 'p')

    # Variables auxiliares para el plot de la grafica
    x_axis = 0
    vel_axis = [0.02]
    w_vel_axis = [0]
    distancias = [wxr[1]]

    # Si la pared esta a la izquierda, calcula el angulo suplementario
    if pared == "i":
        th = math.pi - wxr[2]
    else:
        th = wxr[2]

    # Calculo de la medidad del sensor
    d = wxr[1]/math.cos(norm_pi(th))
    d_ant = d

    # Bulce en el que se recorren los puntos marcados
    while not reached:
        # se calcula la velocidad angular
        w = k1*(dc-d)+k2*(d-d_ant)
        if w > 0:
            wc = min(wmax, w)
        else:
            wc = max(-wmax, w)
        # Aux para el plot de las grafiacas de v y w
        vel_axis.append(v)
        w_vel_axis.append(wc)

        # Se simula el movimiento
        wxr_plus = simubot(np.array([v, wc]), wxr, 0.1)

        # Se comprueba que se ha alcanzado el punto frente a la pared
        if abs(x_meta[0] - wxr_plus[0]) <= dc:
            print(wxr_plus)
            print('Se detiene porque llega')
            reached = True
        d_ant = d
        wxr = wxr_plus
        distancias.append(wxr[1])

        # Si la pared esta a la izquierda, calcula el angulo suplementario
        if pared == "i":
            th = math.pi - wxr[2]
        else:
            th = wxr[2]

        # Calculo de la medidad del sensor
        d = wxr[1]/math.cos(norm_pi(th))
        dibrobot(wxr, 'r', 'p')
        x_axis += 1

    # Muestra el recorrido
    plt.show()

    # Muestra la grafica de velocidad
    plt.plot(range(0, x_axis+1), w_vel_axis, color='red')
    plt.plot(range(0, x_axis+1), vel_axis, color='blue')
    plt.show()

    # Muestra la grafica de las distancias con la pared
    plt.plot(range(0, x_axis+1), distancias, color='blue')
    plt.show()


ej4("d", 0.7, 1.25, 35.0)
ej4("i", 0.7, 1.25, 145.0)

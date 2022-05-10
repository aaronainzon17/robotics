# -*- coding: utf-8 -*-
#!/usr/bin/python

# Standard imports
import cv2
import numpy as np

# Setup default values for SimpleBlobDetector parameters.
params = cv2.SimpleBlobDetector_Params()

# Thresholds
params.minThreshold = 10
params.maxThreshold = 200

# Filter by Area
params.filterByArea = True
# Se ha calculado el area con el minimo diametro obtenido 24mm => A= 452 y se ha dado un margen de 100
params.minArea = 350
# Se ha calculado el area con el maximo diametro obtenido 129mm => A= 13070 y se ha dado un margen de 2000 aprox 
params.maxArea = 150000

# Filter by Circularity
params.filterByCircularity = True 
params.minCircularity = 0.2

# Filter by Color
params.filterByColor = True
# not directly color, but intensity on the channel input
params.blobColor = 255 # 255 para brillantes 0 oscuros 

# Se mantienen a false 
params.filterByConvexity = False
params.filterByInertia = False


# Create a detector with the parameters
ver = (cv2.__version__).split('.')
if int(ver[0]) < 3 :
	detector = cv2.SimpleBlobDetector(params)
else :
	detector = cv2.SimpleBlobDetector_create(params)


# getRedBloobs: devuelve el blob de mayor tamanyo detectado en la imagen frame 
# que se ha pasado por parametro 
def getRedBloobs(frame, HSV_min=(0, 70, 50), HSV_max=(10, 255, 255)):
	
	img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	
	#Se utilizan 2 inRange porque el rojo en HSV se encuentra entre 0-10 y 160-180 
	
	# Limites inferiores (0-10)
	red_0 = np.array([0, 80, 80])
	red_10 = np.array([10, 255, 255])
	
	# Limites superiores (160-180)
	red_160 = np.array([160,80,80])
	red_180 = np.array([179,255,255])

	# Se aplcian los filtros de rango para filtrar por color 
	mask0_10 = cv2.inRange(img_hsv, red_0, red_10)
	mask_160_180 = cv2.inRange(img_hsv, red_160, red_180)

	mask = mask0_10 + mask_160_180

	frame = cv2.bitwise_and(frame, frame, mask=mask)
	
	# Se utiliza el blob detector para calucular los keypoints
	keypoints_red = detector.detect(mask)
	
	# Se elige el blob mas prometedor (mas grande)
	if (len(keypoints_red) != 0):
		biggest = keypoints_red[0]
		for kp in keypoints_red:
			if kp.size > biggest.size:
				biggest = kp 
	else:
		biggest = None

	# Se devuelve el blob mas grande
	return biggest

def detect_red(frame):
	cv2.namedWindow("Capture")
	cv2.imshow("Capture", frame)
	cv2.waitKey(0)
	img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	#Se utilizan 2 inRange porque el rojo en HSV se encuentra entre 0-10 y 160-180 
	
	# Limites inferiores (0-10)
	red_0 = np.array([0, 80, 80])
	red_10 = np.array([10, 255, 255])
	
	# Limites superiores (160-180)
	red_160 = np.array([160,80,80])
	red_180 = np.array([179,255,255])
	
	mask0_10 = cv2.inRange(img_hsv, red_0, red_10)
	mask_160_180 = cv2.inRange(img_hsv, red_160, red_180)

	mask = mask0_10 + mask_160_180
	red_pixels = 0
	for i in range(len(mask[479])):
		if mask[479][i] == 255:
			red_pixels+=1
	print(red_pixels)
	return red_pixels

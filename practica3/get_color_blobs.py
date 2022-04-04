# -*- coding: utf-8 -*-
#!/usr/bin/python

# Standard imports
import cv2
from cv2 import waitKey
import numpy as np;
import time     # import the time library for the sleep function
#../fotosRobot/pelota_cerca_pero_no_mucho.jpg
# Read image
img_BGR = cv2.imread("../fotosRobot/pelota_pinzas.jpg")

# Setup default values for SimpleBlobDetector parameters.
params = cv2.SimpleBlobDetector_Params()

# These are just examples, tune your own if needed
# Change thresholds
params.minThreshold = 10
params.maxThreshold = 200

# Filter by Area
params.filterByArea = True
# Se ha calculado el area con el minimo diametro obtenido 24mm => A= 452 y se ha dado un margen de 100
params.minArea = 350
# Se ha calculado el area con el maximo diametro obtenido 129mm => A= 13070 y se ha dado un margen de 2000 aprox 
params.maxArea = 150000

# Filter by Circularity
# Lo he puesto a false porque sino en pelota_cerca como no es circular porque 
# las pinzas tapan no detectaba la pelota
params.filterByCircularity = False 
params.minCircularity = 0.1 # Innecesario porque es false

# Filter by Color
# No noto al diferencia de ponerlo a true y false 
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

# keypoints on original image (will look for blobs in grayscale)
#keypoints = detector.detect(img_BGR)
# Draw detected blobs as red circles.
# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures
# the size of the circle corresponds to the size of blob
#im_with_keypoints = cv2.drawKeypoints(img_BGR, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
#
## Show blobs
#cv2.imshow("Keypoints on Gray Scale", im_with_keypoints)
#cv2.waitKey(0)
#
## filter certain COLOR channels
#
## Pixels with 100 <= R <= 255, 15 <= B <= 56, 17 <= G <= 50 will be considered red.
## similar for BLUE
#
## BY DEFAULT, opencv IMAGES have BGR format
##redMin = (10, 10, 100)
#redMax = (50, 50, 255)
#
#blueMin=(60, 10, 10)
#blueMax=(255, 100, 100)
#
#mask_red=cv2.inRange(img_BGR, redMin, redMax)
#mask_blue=cv2.inRange(img_BGR, blueMin, blueMax)
#
#
## apply the mask
#red = cv2.bitwise_and(img_BGR, img_BGR, mask = mask_red)
#blue = cv2.bitwise_and(img_BGR, img_BGR, mask = mask_blue)
## show resulting filtered image next to the original one
#cv2.imshow("Red regions", np.hstack([img_BGR, red]))
#cv2.imshow("Blue regions", np.hstack([img_BGR, blue]))
#
## detector finds "dark" blobs by default, so invert image for results with same detector
#keypoints_red = detector.detect(255-mask_red)
#keypoints_blue = detector.detect(255-mask_blue)
#
## documentation of SimpleBlobDetector is not clear on what kp.size is exactly, but it looks like the diameter of the blob.
#prueba = []
#for kp in keypoints_red:
#	print (kp.pt[0], kp.pt[1], kp.size)
#
## Draw detected blobs as red circles.
## cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures
## the size of the circle corresponds to the size of blob
#im_with_keypoints = cv2.drawKeypoints(img_BGR, keypoints_red, np.array([]),
#	(255,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
#im_with_keypoints2 = cv2.drawKeypoints(img_BGR, keypoints_blue, np.array([]),
#	(255,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
#
## Prueba: se aumenta el diametro del circulo donde se encuentran los KeyPoints para hacer A
#for kp in keypoints_red:
#	kp.size += 20
#	print (kp.pt[0], kp.pt[1], kp.size)
#
#im_with_keypoints_A = cv2.drawKeypoints(im_with_keypoints, keypoints_red, np.array([]),
#	(255,0,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
#
## Show mask and blobs found
#cv2.imshow("Keypoints on RED and A", im_with_keypoints_A)
#cv2.waitKey(0)
#
##cv2.imshow("Keypoints on RED", im_with_keypoints)
#cv2.imshow("Keypoints on BLUE", im_with_keypoints2)
#cv2.waitKey(0)


def getRedBloobs(frame, HSV_min=(0, 70, 50), HSV_max=(10, 255, 255)):
	
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

	# Se pueden eliminar los blobs de ruido con erode y dilate

	frame = cv2.bitwise_and(frame, frame, mask=mask)
	
	keypoints_red = detector.detect(mask)
	
	# documentation of SimpleBlobDetector is not clear on what kp.size is exactly, but it looks like the diameter of the blob.
	if (len(keypoints_red) != 0):
		biggest = keypoints_red[0]
		for kp in keypoints_red:
			if kp.size > biggest.size:
				biggest = kp 
	else:
		biggest = None

	
	# Draw detected blobs as red circles.
	# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures
	# the size of the circle corresponds to the size of blob
	
	im_with_keypoints = cv2.drawKeypoints(frame, keypoints_red, np.array([]),
		(255,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

	#cv2.imshow('Capture', im_with_keypoints)

	cv2.startWindowThread()
	cv2.namedWindow("Capture")
	cv2.imshow("Capture", im_with_keypoints)

	#cv2.waitKey(0)
	return biggest
	

	#cv2.waitKey(0)
	#cv2.destroyAllWindows()


cam = cv2.VideoCapture(0)
time.sleep(1)

while(True):
	_, frame = cam.read()       # Se captura un fotograma
	blob = getRedBloobs(frame)  # Se devuelve el blob mas grande
	print('EL bloob esta en', blob.pt[0], blob.pt[1])
	print('El tamanyo del blob es', blob.size)
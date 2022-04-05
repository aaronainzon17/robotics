import cv2
from cv2 import waitKey
import numpy as np;
import time     # import the time library for the sleep function
import picamera
from picamera.array import PiRGBArray
#Parametros para el simple blob detector
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
params.filterByCircularity = True 
params.minCircularity = 0.2 # Innecesario porque es false

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


def getBlobs(frame, HSV_min=(0, 70, 50), HSV_max=(10, 255, 255)):
	
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
	
	im_with_keypoints = cv2.drawKeypoints(frame, keypoints_red, np.array([]),
		(255,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

	cv2.startWindowThread()
	cv2.namedWindow("Capture")
	cv2.imshow("Capture", im_with_keypoints)

	return biggest

cam = picamera.PiCamera()
cam.resolution = (640,480)
cam.framerate = 32 
rawCapture = PiRGBArray(cam, size=(640, 480))

time.sleep(0.1)

while(True):
    cam.capture(rawCapture, format="bgr", use_video_port=True)
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
    frame = rawCapture.array
    blob = getBlobs(frame)  # Se devuelve el blob mas grande

    if blob is not None:
        print('EL bloob esta en', blob.pt[0], blob.pt[1])
        print('El tamanyo del blob es', blob.size)

        if (blob.size > 210) and abs(blob.pt[0] - 320) < 50 and (blob.pt[1] > 240):
            print('Esta en las pinzas')
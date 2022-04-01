# import the opencv library
import cv2
  
  
# define a video capture object
vid = cv2.VideoCapture(0)
  
      
# Capture the video frame
# by frame
while(True):

    _, frame = vid.read()

    # Display the resulting frame
    cv2.imshow('frame', frame)

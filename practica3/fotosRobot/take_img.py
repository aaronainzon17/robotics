# import the opencv library
import cv2
  
  
# define a video capture object
vid = cv2.VideoCapture(0,cv2.CAP_DSHOW)
  
      
# Capture the video frame
# by frame
ret, frame = vid.read()

# Display the resulting frame
cv2.imshow('frame', frame)
# Filename
filename = 'homografia1.jpg'

# Using cv2.imwrite() method
# Saving the image
cv2.imwrite(filename, frame)
cv2.waitKey(0)

  
# After the loop release the cap object
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()
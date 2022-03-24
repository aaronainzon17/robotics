# import the opencv library
import cv2
  
  
# define a video capture object
vid = cv2.VideoCapture(0)
  
      
# Capture the video frame
# by frame
ret, frame = vid.read()

# Display the resulting frame
cv2.imshow('frame', frame)
# Filename
filename = 'foto_bot.jpg'

# Using cv2.imwrite() method
# Saving the image
cv2.imwrite(filename, frame)

  
# After the loop release the cap object
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()
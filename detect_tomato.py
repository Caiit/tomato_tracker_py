import cv2
import numpy as np

# Get the threshold of the image
def getthresholdedimg(hsv):
	red = cv2.inRange(hsv,np.array((0,140, 70)),np.array((3,250,200)))
	return red

# Read the image
# img=cv2.imread("dataset/tomato.jpg")


# Read the video
vid = cv2.VideoCapture("dataset/short.avi")

while(vid.isOpened()):
	_, f = vid.read()
	if f == None : break
	blur = cv2.medianBlur(f,5)
	hsv = cv2.cvtColor(f,cv2.COLOR_BGR2HSV)
	red = getthresholdedimg(hsv)
	erode = cv2.erode(red,None,iterations = 3)
	dilate = cv2.dilate(erode,None,iterations = 10)

# # Convert BGR to HSV
# hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
# red = cv2.inRange(hsv,np.array((0,140, 70)),np.array((3,250,200)))
# erode = cv2.erode(red,None,iterations = 3)
# dilate = cv2.dilate(erode,None,iterations = 10)

	# Find contours
	contours,hierarchy = cv2.findContours(dilate,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

	# Draw a circle around the object
	for cnt in contours:
	    x,y,w,h = cv2.boundingRect(cnt)
	    cx,cy = x+w/2, y+h/2

	    cv2.circle(f,(cx,cy),(w)/2,[255,0,0],2)
	    cv2.circle(f,(cx,cy),2,[255,0,0],2)

	
	cv2.imshow('img', f)
	if cv2.waitKey(80) & 0xFF == ord('q'):
		break

#cv2.waitKey(0)
vid.release()
cv2.destroyAllWindows()

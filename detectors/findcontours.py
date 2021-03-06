import cv2
import numpy as np

# Read the image
# img=cv2.imread("dataset/tomato.jpg")

# Read the video
vid = cv2.VideoCapture("dataset/short.avi")
fourcc = cv2.cv.CV_FOURCC(*'XVID')
out = cv2.VideoWriter('findcontours.avi',fourcc, 10.0, (320,240))

while(vid.isOpened()):
	_, f = vid.read()
	if f == None : break
	hsv = cv2.cvtColor(f,cv2.COLOR_BGR2HSV)
	red = red = cv2.inRange(hsv,np.array((0,140, 70)),np.array((4,250,200)))
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
	    (x,y),radius = cv2.minEnclosingCircle(cnt)
	    center = (int(x),int(y))
	    radius = int(radius)
	    if radius > 20:
		    cv2.circle(f,center,radius,(0,255,0),2)
		    cv2.circle(f,center,2,(0,0,255),2)

	cv2.imshow('findContours', f)
	out.write(f)
	if cv2.waitKey(120) & 0xFF == ord('q'):
		break

vid.release()
out.release()
cv2.waitKey(0)
cv2.destroyAllWindows()

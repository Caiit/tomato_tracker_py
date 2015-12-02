import cv2
import numpy as np

# for a distance of 0.4 meter, the radius of the tomato is 26
radiusToMeters = 26*0.4

# Read the image
img=cv2.imread("dataset/40center.jpg")

hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
red = red = cv2.inRange(hsv,np.array((0,140, 70)),np.array((5,250,200)))
erode = cv2.erode(red,None,iterations = 3)
dilate = cv2.dilate(erode,None,iterations = 10)

# Find contours
contours,hierarchy = cv2.findContours(dilate,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
# Draw a circle around the object
for cnt in contours:
    (x,y),radius = cv2.minEnclosingCircle(cnt)
    center = (int(x),int(y))
    radius = int(radius)
    if radius > 20:
	    cv2.circle(img,center,radius,(0,255,0),2)
	    cv2.circle(img,center,2,(0,0,255),2)

    distance = (radiusToMeters / radius)

cv2.imshow('findContours', img)

cv2.waitKey(0)
cv2.destroyAllWindows()

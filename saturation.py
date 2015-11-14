import cv2
import numpy as np

# Read the video
vid = cv2.VideoCapture("scoring.avi")

while(vid.isOpened()):
	_, f = vid.read()
	if f == None: break

	hsv = cv2.cvtColor(f,cv2.COLOR_BGR2HSV)
	s = hsv[:,1:]
	im = cv2.GaussianBlur(s, (1,1), 0, 0)

	edges = cv2.Canny(im,60,180,apertureSize = 3,  L2gradient=True)

	lines = cv2.HoughLinesP(edges,10,np.pi/2,100, minLineLength = 10, maxLineGap = 5)
	if lines != None: 
		for x1,y1,x2,y2 in lines[0]:
			cv2.line(f,(x1,y1),(x2,y2),(0,255,0),2)

	cv2.imshow('img', f)
	if cv2.waitKey(100) & 0xFF == ord('q'):
		break

#cv2.waitKey(0)
vid.release()

cv2.waitKey(0)
cv2.destroyAllWindows()
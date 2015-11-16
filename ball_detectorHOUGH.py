import cv2
import cv2.cv as cv
import numpy as np

# Read the video
vid = cv2.VideoCapture("football/scoring.avi")

while(vid.isOpened()):
    _, f = vid.read()
    if f == None: break

    hsv = cv2.cvtColor(f,cv2.COLOR_BGR2HSV)
    s = hsv[:,1:]
    im = cv2.GaussianBlur(s, (13,13), 0, 0)
    edges = cv2.Canny(im,60,180,apertureSize = 3,  L2gradient=True)

    contours,hierarchy = cv2.findContours(edges,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(f, contours, -1, (255,0,0), 3)

    # circles = cv2.HoughCircles(edges,cv.CV_HOUGH_GRADIENT,2,20,param1=50,param2=60,minRadius=0,maxRadius=30)
    # backtorgb = cv2.cvtColor(edges,cv2.COLOR_GRAY2RGB)
    # if circles != None:
    #     circles = np.uint16(np.around(circles))
    #     for i in circles[0,:]:
    #         # draw the outer circle
    #         cv2.circle(backtorgb,(i[0],i[1]),i[2],(0,255,0),2)
    #         # draw the center of the circle
    #         cv2.circle(backtorgb,(i[0],i[1]),2,(0,0,255),3)

    cv2.imshow('img', f)
    if cv2.waitKey(100) & 0xFF == ord('q'):
    	break

#cv2.waitKey(0)
vid.release()

cv2.waitKey(0)
cv2.destroyAllWindows()

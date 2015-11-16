import cv2, sys
import cv2.cv as cv
import numpy as np

vid = cv2.VideoCapture("football/scoring.avi")

while(vid.isOpened()):
    _, f = vid.read()
    if f == None : break

    img = cv2.medianBlur(f,5)
    cimg = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    circles = cv2.HoughCircles(cimg,cv.CV_HOUGH_GRADIENT,2,20,param1=50,param2=60,minRadius=0,maxRadius=30)

    if circles != None:
        circles = np.uint16(np.around(circles))
        for i in circles[0,:]:
            # draw the outer circle
            cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
            # draw the center of the circle
            cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)


    cv2.imshow('detected circles',cimg)
    if cv2.waitKey(80) & 0xFF == ord('q'):
        break

vid.release()
cv2.destroyAllWindows()

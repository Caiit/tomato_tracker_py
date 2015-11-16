import cv2, sys
import cv2.cv as cv
import numpy as np

vid = cv2.VideoCapture("football/scoring.avi")

while(vid.isOpened()):
    _, f = vid.read()
    if f == None : break

    # img = cv2.medianBlur(f,5)
    # cimg = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    # circles = cv2.HoughCircles(cimg,cv.CV_HOUGH_GRADIENT,2,20,param1=50,param2=60,minRadius=0,maxRadius=30)

    edges = cv2.Canny(f,60,180,apertureSize = 3,  L2gradient=True)

    contours,hierarchy = cv2.findContours(f,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(f, contours, -1, (255,0,0), 3)


    detector = cv2.SimpleBlobDetector()

    # Detect blobs.
    keypoints = detector.detect(f)

    # Draw detected blobs as red circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    im_with_keypoints = cv2.drawKeypoints(f, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)


    cv2.imshow('detected circles',im_with_keypoints)
    if cv2.waitKey(80) & 0xFF == ord('q'):
        break

vid.release()
cv2.destroyAllWindows()

import cv2
import numpy as np

# Read the video
vid = cv2.VideoCapture("football/rolling_ball.avi")

while(vid.isOpened()):
    _, f = vid.read()
    if f == None: break

    hsv = cv2.cvtColor(f,cv2.COLOR_BGR2HSV)
    s = hsv[:,1:]
    im = cv2.GaussianBlur(s, (13,13), 0, 0)

    edges = cv2.Canny(im,60,180,apertureSize = 3,  L2gradient=True)
    torgb = cv2.cvtColor(edges, cv2.COLOR_GRAY2RGB)

    contours,hierarchy = cv2.findContours(edges,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(f, contours, -1, (255,0,0), 3)

    params = cv2.SimpleBlobDetector_Params()
    params.filterByCircularity = True
    params.minCircularity = 0.1

    detector = cv2.SimpleBlobDetector(params)

    # Detect blobs.
    keypoints = detector.detect(f)

    # Draw detected blobs as yellow circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    im_with_keypoints = cv2.drawKeypoints(f, keypoints, np.array([]), (0,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)




    cv2.imshow('detected circles',im_with_keypoints)
    if cv2.waitKey(100) & 0xFF == ord('q'):
    	break

#cv2.waitKey(0)
vid.release()

cv2.waitKey(0)
cv2.destroyAllWindows()

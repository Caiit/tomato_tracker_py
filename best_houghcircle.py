import cv2
import cv2.cv as cv
import numpy as np

# Read the video
vid = cv2.VideoCapture("dataset/short.avi")
fourcc = cv2.cv.CV_FOURCC(*'XVID')
out = cv2.VideoWriter('houghcircles.avi',fourcc, 10.0, (320,240))
while(vid.isOpened()):
    _, f = vid.read()
    if f == None: break
    # Hough Circles
    hsv = cv2.cvtColor(f,cv2.COLOR_BGR2HSV)
    # red = cv2.inRange(hsv,np.array((0,140, 70)),np.array((3,250,200)))
    im = cv2.GaussianBlur(hsv, (15,15), 0, 0)
    edges = cv2.Canny(im,15,30,apertureSize = 3,  L2gradient=True)
    circles = cv2.HoughCircles(edges,cv.CV_HOUGH_GRADIENT,1,30,
                                param1=30,param2=22,minRadius=20,maxRadius=60)

    # cv2.imshow("cimg",edges)
    # print(circles)

    best_circle = None
    best_roi = 0;
    min_hsv = (20, 20, 70)
    max_hsv = (40, 55, 200)
    if circles is not None:
        circles = np.uint16(np.around(circles))

        for i in circles[0,:]:
            # 4 Points of Region of interest square
            x_min = i[0] - i[2]
            x_max = i[0] + i[2]
            y_min = i[1] - i[2]
            y_max = i[1] + i[2]

            # For now just take the square
            roi = f[y_min:y_max, x_min:x_max]

            # Calculate average
            roi_sum = np.sum(np.sum(roi, axis=0), axis=0) / (np.shape(roi)[0]*np.shape(roi)[1])
            # print roi_sum

            # We know that the tomato is red
            if (roi_sum > min_hsv).all() and (roi_sum < max_hsv).all():
                best_val = roi_sum[2]
                best_circle = i
                best_roi = roi


            # cv2.circle(f,(i[0],i[1]),i[2],(255,255,0),2)


            #sums.append(roi_sum)
    if not best_circle == None:
        cv2.circle(f,(best_circle[0],best_circle[1]),best_circle[2],(0,255,0),2)
        # draw the center of the circle
        cv2.circle(f,(best_circle[0],best_circle[1]),2,(0,0,255),3)
    cv2.imshow( "ROI", f)
    out.write(f)
    if cv2.waitKey(120) & 0xFF == ord('q'):
    	break



vid.release()
out.release()
cv2.waitKey(0)
cv2.destroyAllWindows()

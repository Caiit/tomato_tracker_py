import cv2.cv as cv
import cv2
# import pySaliencyMap
import numpy as np

img = cv2.imread('dataset/short.avi')
# img = img.astype(np.uint8)
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
ret,thresh = cv2.threshold(gray,127,255,0)

cv2.imshow('detected circles',gray)
# imgsize = img.shape
# img_width  = imgsize[1]
# img_height = imgsize[0]
# sm = pySaliencyMap.pySaliencyMap(img_width, img_height)
# map = sm.SMGetSM(img)
# cv2.imshow("input",  img)
# cv2.imshow("output", map)
# dst = cv2.addWeighted(img,0.7,map,0.3,0)
# cv2.imshow('lala',dst)
# gray = cv2.cvtColor(map,cv2.COLOR_RGB2GRAY)


# Find contours
contours,hierarchy = cv2.findContours(thresh,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
cv2.drawContours(img, contours, -1, (255,0,0), 3)


# cv2.imshow('img', map)
# circles = cv2.HoughCircles(slice1Copy,cv.CV_HOUGH_GRADIENT,1,250, param1=50,param2=30,minRadius=0,maxRadius=0)

# circles = np.uint16(np.around(circles))
# for i in circles[0,:]:
#     # draw the outer circle
#     cv2.circle(img,(i[0],i[1]),i[2],(0,255,0),2)
#     # draw the center of the circle
#     cv2.circle(img,(i[0],i[1]),2,(0,0,255),3)

# # cv2.imshow('detected circles',img)
cv2.imshow('detected circles',img)
cv2.waitKey(0)
cv2.destroyAllWindows()

# plt.show()

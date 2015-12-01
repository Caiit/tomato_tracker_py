import cv2
import numpy as np

# Read the image
# img=cv2.imread("dataset/tomato.jpg")


# Read the video
vid = cv2.VideoCapture("dataset/short.avi")
fourcc = cv2.cv.CV_FOURCC(*'XVID')
out = cv2.VideoWriter('blob.avi',fourcc, 10.0, (320,240))

while(vid.isOpened()):
	_, f = vid.read()
	if f == None : break

	# Setup SimpleBlobDetector parameters.
	params = cv2.SimpleBlobDetector_Params()
	 
	# Filter by Area.
	params.filterByArea = True
	params.minArea = 500
	 
	# Filter by Circularity
	params.filterByCircularity = True
	params.minCircularity = 0.5
	 
	# Filter by Convexity
	params.filterByConvexity = True
	params.minConvexity = 0.5
	
	# Set up the detector with the parameters.
	detector = cv2.SimpleBlobDetector(params)

	# Detect blobs.
	keypoints = detector.detect(f)
	 
	# Draw detected blobs as red circles.
	# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
	im_with_keypoints = cv2.drawKeypoints(f, keypoints, np.array([]), (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
	 
	cv2.imshow('blobs', im_with_keypoints)
	out.write(im_with_keypoints)
	if cv2.waitKey(120) & 0xFF == ord('q'):
		break

vid.release()
out.release()
cv2.waitKey(0)
cv2.destroyAllWindows()

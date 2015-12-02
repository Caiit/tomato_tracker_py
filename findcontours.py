import cv2
import numpy as np
from naoqi import ALProxy
import vision_definitions

IP = "192.168.1.49"
PORT = 9559

# Create proxy to nao
print "Creating ALVideoDevice proxy to ", IP
camProxy = ALProxy("ALVideoDevice", IP, PORT)

# Register a Generic Video Module
resolution = vision_definitions.kQVGA
colorSpace = vision_definitions.kYUVColorSpace
fps = 30

# create image
print(resolution)
width = 320
height = 240
image = np.zeros((height, width, 3), np.uint8)

nameId = camProxy.subscribe("python_GVM", resolution, colorSpace, fps)
print nameId

print 'getting images in remote'
while True:
	nao_image = camProxy.getImageRemote(nameId)
	if nao_image == None:
		print 'cannot capture.'
	elif nao_image[6] == None:
		print 'no image data string.'
	else:
		# translate value to mat
		values = map(ord, list(nao_image[6]))
		i = 0
		for y in range(0, height):
			for x in range(0, width):
				image.itemset((y, x, 0), values[i + 0])
				image.itemset((y, x, 1), values[i + 1])
				image.itemset((y, x, 2), values[i + 2])
				i += 3
		print("updated image")
		cv2.imshow("nao_image",image)
		# cv2.waitKey(1)


camProxy.unsubscribe(nameId)

# Read the image
# img=cv2.imread("dataset/tomato.jpg")

<<<<<<< HEAD



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
=======
#
# # Read the video
# vid = cv2.VideoCapture("dataset/short.avi")
# fourcc = cv2.cv.CV_FOURCC(*'XVID')
# out = cv2.VideoWriter('findcontours.avi',fourcc, 10.0, (320,240))
#
# while(vid.isOpened()):
# 	_, f = vid.read()
# 	if f == None : break
# 	blur = cv2.medianBlur(f,5)
# 	hsv = cv2.cvtColor(f,cv2.COLOR_BGR2HSV)
# 	red = red = cv2.inRange(hsv,np.array((0,140, 70)),np.array((3,250,200)))
# 	erode = cv2.erode(red,None,iterations = 3)
# 	dilate = cv2.dilate(erode,None,iterations = 10)
#
# # # Convert BGR to HSV
# # hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
# # red = cv2.inRange(hsv,np.array((0,140, 70)),np.array((3,250,200)))
# # erode = cv2.erode(red,None,iterations = 3)
# # dilate = cv2.dilate(erode,None,iterations = 10)
#
# 	# Find contours
# 	contours,hierarchy = cv2.findContours(dilate,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
# 	# Draw a circle around the object
# 	for cnt in contours:
# 	    (x,y),radius = cv2.minEnclosingCircle(cnt)
# 	    center = (int(x),int(y))
# 	    radius = int(radius)
# 	    if radius > 25:
# 		    cv2.circle(f,center,radius,(0,255,0),2)
# 		    cv2.circle(f,center,2,(0,0,255),2)
#
# 	cv2.imshow('findContours', f)
# 	out.write(f)
# 	if cv2.waitKey(120) & 0xFF == ord('q'):
# 		break
#
# vid.release()
# out.release()
>>>>>>> ea2a4fba0ad334c479ad937d42ac2191552d5655

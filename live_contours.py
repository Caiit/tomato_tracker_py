import cv2
import numpy as np
from naoqi import ALProxy
import vision_definitions

IP = "192.168.0.101"
PORT = 9559

# Create proxy to nao
print "Creating ALVideoDevice proxy to ", IP
camProxy = ALProxy("ALVideoDevice", IP, PORT)

# Register a Generic Video Module
resolution = vision_definitions.kQVGA
colorSpace = vision_definitions.kHSVColorSpace
fps = 30

# create image
width = 320
height = 240
image = np.zeros((height, width, 3), np.uint8)

# for a distance of 0.4 meter, the radius of the tomato is 26
radiusToMeters = 26*0.4
# the width of the real tomato is 0.05 meter, on a distance of 0.40 meter it is 46 pixels
pixToMeters = 46*0.4
pixToMeters = 0.05/46*0.4
# the tomato lies always on a height of 0.35 meters
z = 0.35

nameId = camProxy.subscribe("python_GVM", resolution, colorSpace, fps)
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
				image.itemset((y, x, 0), values[i + 0]) # H
				image.itemset((y, x, 1), values[i + 1]) # S
				image.itemset((y, x, 2), values[i + 2]) # V
				i += 3
		print "updated image",type(image)

		# Prepare image
		blur = cv2.medianBlur(image,5)
		red = cv2.inRange(blur,np.array((0,140, 60)),np.array((10,250,200)))
		erode = cv2.erode(red,None,iterations = 3)
		dilate = cv2.dilate(erode,None,iterations = 10)

		# Find contours
		contours,hierarchy = cv2.findContours(dilate,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
		# Draw a circle around the object
		# print(contours)
		for cnt in contours:
			(x,y),radius = cv2.minEnclosingCircle(cnt)
			center = (int(x),int(y))
			radius = int(radius)
			if radius > 20:
				cv2.circle(image,center,radius,(0,255,0),2)
				cv2.circle(image,center,2,(0,0,255),2)

			x_distance = (radiusToMeters / radius)
			if x < 160:
				y = pixToMeters*(160-x)
			else:
				y = pixToMeters*(x-160)

			x_distance = (radiusToMeters / radius)
			print "X_distance:",x_distance
			print "Y_offset:",y

		cv2.imshow('findContours', image)
		if cv2.waitKey(120) & 0xFF == ord('q'):
			break

camProxy.unsubscribe(nameId)
cv2.destroyAllWindows()

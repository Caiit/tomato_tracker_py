#!/usr/bin/env python
import sys
import time
import rospy
import cv2
import cv2.cv as cv
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

ARM_LENGTH = 0.40

class tomatoDetector(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.pixToMeters = 0.05/(26*2)
        self.imageSub = rospy.Subscriber("nao_robot/camera/top/camera/image_raw",Image,self.callback)
        self.pointPub = rospy.Publisher("point", Point, queue_size=1 )
        self.walkPub = rospy.Publisher("/cmd_vel", Twist, queue_size=1 )
        rospy.loginfo("Initialised tomato detector")

    def callback(self,data):
        rospy.loginfo("Received data")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        point = self.searchForTomato(cv_image)

        # Create twist    
        twist = Twist()

        if point is not None:
            twist = self.walkToTomato(point, twist)
        else:
            rospy.loginfo("No tomato, turn right and search again")
            # twist.linear.x = 0
            # twist.linear.y = 0
            twist.angular.z = -0.5
            # findTomato()

        # publish twist    
        rate = rospy.Rate(1) # 10hz
        # while not rospy.is_shutdown():
            # rospy.loginfo(point)
        self.walkPub.publish(twist)
        rate.sleep()

    # Check if a tomato is in the image, if so return the point, if not return None
    def searchForTomato(self, img):
        # Prepare image
        blur = cv2.medianBlur(img,5)
        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        red = cv2.inRange(hsv,np.array((0, 140, 180)),np.array((3, 250, 240)))
        erode = cv2.erode(red,None,iterations=3)
        dilate = cv2.dilate(erode,None,iterations=10)

        # Find contours
        contours,hierarchy = cv2.findContours(dilate,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

        # for a distance of 0.4 meter, the radius of the tomato is 26
        radiusToMeters = 26*0.4
        # the width of the real tomato is 0.05 meter, on a distance of 0.40 meter its radius is 26 pixels
        pixToMeters = 0.05/(26*2)
        # the tomato lies always on a height of 0.35 meters
        z = 0.35
        # Get the x and y position of the tomato
        tomato = False
        for cnt in contours:
            (x,y),radius = cv2.minEnclosingCircle(cnt)
            center = (int(x),int(y))
            radius = int(radius)
            if radius > 10:
                # draw circle
                # cv2.circle(hsv,center,radius,(0,255,0),2)
                # cv2.circle(hsv,center,2,(0,0,255),2)
                tomato = True
                y = pixToMeters*(160-x)
                x = (radiusToMeters / radius)
                # print "X_distance:", x
                # print "Y_offset:", y
        
        # cv2.imshow('img', hsv)

        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        if tomato:
            # Create point
            point = Point()
            point.x = x
            point.y = y
            point.z = z
            return point
        # else:
        #     return None

    def walkToTomato(self, point, twist):
        twist.angular.z = 0 
        rospy.loginfo(point)
        rospy.loginfo('Walk to tomato')
        if point.x < ARM_LENGTH and point.y > -0.05 and point.y < 0.05:
            rospy.loginfo("go to moveit")
            twist.linear.x = 0
            twist.linear.y = 0
            # publish point    
            # rate = rospy.Rate(10) # 10hz
            while not rospy.is_shutdown():
                # rospy.loginfo(point)
                self.pointPub.publish(point)
                # rate.sleep()
        elif point.x < ARM_LENGTH and point.y < -0.05:
            rospy.loginfo("walk right")
            twist.linear.x = point.x-ARM_LENGTH+0.05
            twist.linear.y = point.y
            # findTomato()
        elif point.x < ARM_LENGTH and point.y > 0.05:
            rospy.loginfo("walk left")
            twist.linear.x = point.x-ARM_LENGTH+0.05
            twist.linear.y = point.y
            # findTomato()
        elif point.x > ARM_LENGTH and point.y < -0.05:
            rospy.loginfo("walk forward right")
            twist.linear.x = point.x-ARM_LENGTH+0.05
            twist.linear.y = point.y
            # findTomato()
        elif point.x > ARM_LENGTH and point.y > 0.05:
            rospy.loginfo("walk forward left")
            twist.linear.x = point.x-ARM_LENGTH+0.05
            twist.linear.y = point.y
            # findTomato()
        else:
            rospy.loginfo("walk forward")
            twist.linear.x = point.x-ARM_LENGTH-0.05
            twist.linear.y = point.y
            # findTomato()
        return twist

def main():
    bd = tomatoDetector()
    rospy.init_node('tomatoDetector', anonymous=True)
    rospy.loginfo("Starting tomato detector")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
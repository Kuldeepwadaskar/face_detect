#!/usr/bin/env python

import cv2
import numpy as np
import rospy
#Import bridge to convert open cv frames into ros frames
from cv_bridge import CvBridge
import Adafruit_PCA9685
from sensor_msgs.msg import Image

pwm = Adafruit_PCA9685.PCA9685(address=0x40, busnum=4)
cap = cv2.VideoCapture(0)

# Set camera resolution
cap.set(3, 480)
cap.set(4, 320)


ret, img = cap.read()
rows, cols, _ = img.shape
x_medium = int(cols / 2)
center = int(cols / 2)
position = 200 # degrees 
pwm.set_pwm_freq(45)
face_cascade = cv2.CascadeClassifier('/home/ubuntu/opencv-3.4.4/data/haarcascades_cuda/haarcascade_frontalface_default.xml')
while 1: 
        #Initializing publisher
        pub = rospy.Publisher("frames", Image, queue_size=10)
        rospy.init_node('stream_publisher', anonymous=True)
        rate = rospy.Rate(10)
	# Reads frames from a camera
	ret, img = cap.read() 
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

	# Detects faces of different sizes in the input image
	faces = face_cascade.detectMultiScale(gray, 1.3, 5)
	for (x,y,w,h) in faces:
		# To draw a rectangle in a face 
		cv2.rectangle(img,(x,y),(x+w,y+h),(255,255,0),2) 
		roi_gray = gray[y:y+h, x:x+w]
		roi_color = img[y:y+h, x:x+w]
        	x_medium = int((x + x + w) / 2)

        	break
       #cv2.line(img, (x_medium, 0), (x_medium, 480), (0, 255, 0), 2)
       #cv2.imshow('img',img)
        bridge= CvBridge() 
        #Encoding bgr8: CV_8UC3 color image with blue-green-red color order    
        ros_image = bridge.cv2_to_imgmsg(img, "bgr8")
        pub.publish(ros_image)  

    	key = cv2.waitKey(1)

    

    	if key == 27:

          break

    # Move servo motor

    	if x_medium < center -30:

        	position += 1.5

    	elif x_medium > center + 30:

        	position -= 1.5

        pwm.set_pwm(0,0, position)



cap.release()

cv2.destroyAllWindows()        
  

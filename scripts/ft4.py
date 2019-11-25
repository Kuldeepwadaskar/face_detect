#!/usr/bin/env python
import rospy
import time
import cv2
from cv_bridge import CvBridge
import array
import numpy as np
from sensor_msgs.msg import Image
from picamera.array import PiRGBArray
# from pisoc import *
import Adafruit_PCA9685

pwm = Adafruit_PCA9685.PCA9685(address=0x40, busnum=4)

pwm.set_pwm_freq(25)

pwm.set_pwm(0, 0,140)

pwm.set_pwm(1, 0,100)

face_cascade = cv2.CascadeClassifier('/home/ubuntu/opencv-3.4.1/data/haarcascades/haarcascade_frontalface_default.xml')
cap = cv2.VideoCapture(0)

cap.set(3, 320)

cap.set(4, 240)



_, img = cap.read()

rows, cols, _ = img.shape

x_medium = int(cols / 2)

center = int(cols / 2)

pos_x = 100
pos_y = 140

while True:
    #Initializing publisher
    pub = rospy.Publisher("/usb_cam/image_raw", Image, queue_size=32)
    rospy.init_node('stream_publisher', anonymous=True)
    rate = rospy.Rate(32)

    # Reads frames from a camera
    ret, img = cap.read()
    # Convert to gray scale of each frames
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Detects faces of different sizes in the input image
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)

    for rect in faces:
        (x,y,w,h) = rect
        cv2.rectangle(faces,(x,y),(x+w,y+h),(115,255,0),2)
        x_medium = int((x + x + w) / 2)
        break
    #cv2.line(img, (x_medium, 0), (x_medium, 340), (0, 255, 0), 2)

    #cv2.imshow("Frame", frame)

    bridge= CvBridge()
    #Encoding bgr8: CV_8UC3 color image with blue-green-red color order 
    ros_image = bridge.cv2_to_imgmsg(img, "bgr8")
    pub.publish(ros_image)


    key = cv2.waitKey(1)
    if key == 27:

        break
    if x_medium < center - 10:

        pos_x += 3

    elif x_medium > center + 10:

        pos_x -= 3



    pwm.set_pwm(1, 0, pos_x)
    # Move servo motor
   # for pos_x in range(50 , 150):

	#if x_medium < center - 10:
	 #  pos_x += 5
        #elif x_medium > center + 10:
         #  pos_x -= 5
        #else:
         #  pwm.set_pwm(1, 0,  pos_x)
	#break
   # try:
    #  rospy.spin()
   # except KeyboardInterrupt:
    #  print("Shutting down")

cap.release()

cv2.destroyAllWindows()




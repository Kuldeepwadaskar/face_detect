#!/usr/bin/env python

import rospy
import time
import cv2
from cv_bridge import CvBridge
import array
from sensor_msgs.msg import Image
from picamera.array import PiRGBArray
# from pisoc import *
import Adafruit_PCA9685
#pwm = Adafruit_PCA9685.PCA9685()
pwm = Adafruit_PCA9685.PCA9685(address=0x40, busnum=4)

#    position=position-delta.x
#position=200
#FRAME_W = 180
#FRAME_H = 100
#cam_pan = 90
#cam_tilt = 60
pwm.set_pwm_freq(45)
#pwm.set_pwm(0, 0,120)
#pwm.set_pwm(1, 0,120)



class Point(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y

def Track(pan, tilt, center, target = Point(100, 240), threshold = Point(16, 24), delta = Point(20, 20)):
    global position
    position=240
    if (center.x < target.x + threshold.x):
        position=position-delta.x
        print("xup = ", position)
        pwm.set_pwm(1, 0,position)

        # pan.SetAngle(pan.ReadAngle() - delta.x)
    elif (center.x > target.x - threshold.x):
        position=position+delta.x
        print("yup = ", position)
        pwm.set_pwm(1, 0,position)

        # pan.SetAngle(pan.ReadAngle() + delta.x)
    if (center.y > target.y + threshold.y):
        position=position+delta.x
        print("xlr = ", position)
        pwm.set_pwm(0, 0, position)
        # tilt.SetAngle(tilt.ReadAngle() + delta.y)
    elif (center.y < target.y - threshold.y):
        position=position-delta.x
        print("ylr = ", position)
        pwm.set_pwm(0, 0,position)
    #elif:
     #   pwm.set_pwm(0, 0,200)
      #  pwm.set_pwm(1, 0,200)
        # tilt.SetAngle(tilt.ReadAngle() - delta.y)
    #print ("x = %d " % center.x)
    #print ("y = %d " % center.y)

if __name__ == "__main__":
    # PiSoC(log_level = 'debug')
    pan= pwm.set_pwm(0, 0, 240)
    #print (pan)
    tilt=pwm.set_pwm(1,0, 190)
    face_cascade = cv2.CascadeClassifier('/home/ubuntu/lbpcascade_frontalface.xml')

    cap = cv2.VideoCapture(0)
    cap.set(3, 320)
    cap.set(4, 240)
    try:
      while 1:
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


       	for (x, y, w, h) in faces:
           cv2.rectangle(img,(x,y),(x+w,y+h),(255,255,0),2)
    	   Track(pan, tilt, Point((x + x + w)/2.0, (y + y + h)/2.0))
           break
	for (x, y, w, h) in faces:
	   rec = cv2.rectangle(img,(x,y),(x+w,y+h),(255,255,0),2)
	   if rec is None:
            time.sleep(0.5)
            pwm.set_pwm(0, 0, 240)
            pwm.set_pwm(1, 0, 190)

	bridge= CvBridge() 
        #Encoding bgr8: CV_8UC3 color image with blue-green-red color order    
        ros_image = bridge.cv2_to_imgmsg(img, "bgr8")
        pub.publish(ros_image)
        rate.sleep()

       # cv2.imshow("Result", image)
        key = cv2.waitKey(1) 

        if key == 27:
           break

    except KeyboardInterrupt:
    	print ("User quit! Moving servo to middle position.")
	cap.release()
        cv2.destroyAllWindows() 


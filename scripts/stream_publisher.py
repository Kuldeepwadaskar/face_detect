#!/usr/bin/env python
##Python 2.x program for face detection
#With in the environment
#declare -x ROSLISP_PACKAGE_DIRECTORIES=""
#declare -x ROS_DISTRO="kinetic"
#declare -x ROS_ETC_DIR="/opt/ros/kinetic/etc/ros"
#declare -x ROS_MASTER_URI="http://localhost:11311"
#declare -x ROS_PACKAGE_PATH="/opt/ros/kinetic/share"
#declare -x ROS_ROOT="/opt/ros/kinetic/share/ros"
#declare -x ROS_VERSION="1"
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
##Copyright (c) 2018, MUHAMMED RULSHID.S, Licensed under the
#Educational Community License, Version 2.0 (the "License"); you may
#not use this file except in compliance with the License. You may
#obtain a copy of the License at

#http://www.osedu.org/licenses/ECL-2.0

#Unless required by applicable law or agreed to in writing,
#software distributed under the License is distributed on an "AS IS"
#BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express
#or implied. See the License for the specific language governing
#permissions and limitations under the License.
# OpenCV program to detect face in real time
# Import libraries of python OpenCV 
# Where its functionality resides

import rospy

import cv2
#Import bridge to convert open cv frames into ros frames
from cv_bridge import CvBridge

from sensor_msgs.msg import Image

## Trained XML file for detecting face with its path
#If the xml file&this program are in the same directory path not required
face_cascade = cv2.CascadeClassifier('/home/ubuntu/opencv-3.4.4/data/haarcascades_cuda/haarcascade_frontalface_default.xml')
# Trained XML file for detecting eyes
# eye_cascade = cv2.CascadeClassifier('/home/rulshid/opencv-3.1.0/data/haarcascades_cuda/haarcascade_eye.xml')
# Capture frames from a camera
cap = cv2.VideoCapture(0)
# loop runs if capturing has been initialized.
def make_480p():
    cap.set(3, 640)
    cap.set(4, 480)

def change_res(width, height):
    cap.set(3, width)
    cap.set(4, height)

change_res(160, 120)

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
	faces = face_cascade.detectMultiScale(gray, 1.1, 5)

	for (x,y,w,h) in faces:
		# To draw a rectangle in a face 
		cv2.rectangle(img,(x,y),(x+w,y+h),(255,255,0),2) 
		roi_gray = gray[y:y+h, x:x+w]
		roi_color = img[y:y+h, x:x+w]

		# # Detects eyes of different sizes in the input image
		# eyes = eye_cascade.detectMultiScale(roi_gray) 

		# #To draw a rectangle in eyes
		# for (ex,ey,ew,eh) in eyes:
		# 	cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,127,255),2)

	##Use the belw comment if you need ti display an image in a window
	
        #Give the frames to ros Environment 
        bridge= CvBridge() 
        #Encoding bgr8: CV_8UC3 color image with blue-green-red color order    
        ros_image = bridge.cv2_to_imgmsg(img, "bgr8")
        pub.publish(ros_image)
        rate.sleep()

	# Wait for Esc key to stop
	if cv2.waitKey(10) & 0xff == ord('q'):
		break

# Close the window
cap.release()

# De-allocate any associated memory usage
cv2.destroyAllWindows() 

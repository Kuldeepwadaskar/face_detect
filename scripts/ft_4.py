#!/usr/bin/env python
import rospy
import time
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
cap = cv2.VideoCapture(0)
cap.set(3, 480)
cap.set(4, 320)
_, img = cap.read()
face_cascade = cv2.CascadeClassifier('/home/ubuntu/opencv-3.4.1/data/haarcascades_cuda/haarcascade_frontalface_default.xml')
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
    _, contours, _ = cv2.findContours(faces, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=lambda x:cv2.contourArea(x), reverse=True)
    for cnt in contours:
        (x, y, w, h) = cv2.boundingRect(cnt)

        x_medium = int((x + x + w) / 2)
        break
    # cv2.imshow("Frame", img)

    bridge= CvBridge()
    #Encoding bgr8: CV_8UC3 color image with blue-green-red color order 
    ros_image = bridge.cv2_to_imgmsg(img, "bgr8")
    pub.publish(ros_image)
    key = cv2.waitKey(1)
    if key == 27:
        break
    try:
      rospy.spin()
    except KeyboardInterrupt:
      print("Shutting down")
cap.release()
cv2.destroyAllWindows()

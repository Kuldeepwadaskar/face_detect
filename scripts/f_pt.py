#!/usr/bin/env python
import rospy
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
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
#pwm.set_pwm(0, 0,position)
if __name__ == "__main__":
    # PiSoC(log_level = 'debug')
    pan= pwm.set_pwm(0, 0, 240)
    #print (pan)
    tilt=pwm.set_pwm(1,0, 190)
    # pan = Servo(0, max_angle = 320)
    # tilt = Servo(1, max_angle = 240)
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera , size = camera.resolution)

    face_cascade = cv2.CascadeClassifier('/home/ubuntu/lbpcascade_frontalface.xml')

    scale = (camera.resolution[0]/320.0, camera.resolution[1]/240.0)

    time.sleep(0.1)
 #   pan.Start()
#    tilt.Start()

    for frame in camera.capture_continuous(rawCapture, format = 'bgr', use_video_port = True):
        image = frame.array
        rec = 0
        resized = cv2.resize(image, (320, 240))
        gray = cv2.cvtColor(resized,cv2.COLOR_BGR2GRAY)

        faces = face_cascade.detectMultiScale(gray, 1.1, 5)
        if len(faces) > 0:
            for (x, y, w, h) in faces:
                Track(pan, tilt, Point((x + x + w)/2.0, (y+ y + h)/2.0))
                break
        faces_resized = [(int(scale[0]*x), int(scale[1]*y), int(scale[0]*w), int(scale[1]*h)) for (x, y, w, h) in faces]
        for (x,y,w,h) in faces_resized:
            rec=cv2.rectangle(image,(x,y),(x+w,y+h),(255,255,0),2)

        if rec is None:
            time.sleep(0.5)
            pwm.set_pwm(0, 0, 240)
            pwm.set_pwm(1, 0, 190)

       # cv2.imshow("Result", image)
        key = cv2.waitKey(1) & 0xFF

        rawCapture.truncate(0)

        if key == ord('q') or key == 27:
            break
  #  pan.Stop()
   # tilt.Stop()


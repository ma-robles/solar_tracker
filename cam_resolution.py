from __future__ import print_function
from __future__ import division
import picamera
import picamera.array
import cv2
import datetime
import time

def getpos(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDBLCLK:
        print(x,y)
cv2.namedWindow('image')
cv2.setMouseCallback('image', getpos)
camera = picamera.PiCamera(
        resolution=(1296,972),
        framerate = 5,
        )
camera.vflip = True
camera.iso = 1600
camera.shutter_speed = 5000
#while (True):
with picamera.array.PiRGBArray(camera) as stream:
    for frame in camera.capture_continuous(
        stream,
        format='bgr',
        use_video_port = True,
        ):
        image = frame.array
        #cv2.imshow('image', image)
        #key=cv2.waitKey(1)&0xff
        #stream.seek(0)
        stream.truncate(0)
        print (datetime.datetime.now())
        #if key == 'q':
        #    break

from __future__ import print_function
from __future__ import division
import time
import picamera
import picamera.array
import cv2
import numpy as np
from datetime import datetime
import math

class detector():
    def pyCamConfig(self,
            camera, 
            exposure='off', 
            awb_mode= 'off',
            iso=1000,
            sSpeed=4000
            ):
        '''config pycam'''
        camera.exposure_mode = exposure
        camera.awb_mode = awb_mode
        camera.iso = iso
        camera.shutter_speed= sSpeed

    def myfilter(self, image, kernel):
        '''filter the image'''
        return cv2.morphologyEx(image ,cv2.MORPH_OPEN, kernel)
        #return cv2.morphologyEx(image ,cv2.MORPH_CLOSE, kernel)
        #return cv2.erode(image, kernel)

    def meanXY(self, image):
        y,x = image.nonzero()
        try:
            return(int(x.mean()), int(y.mean()), len(x))
        except ValueError:
            return(None,None,None)

    def tocenter(self, point, size):
        x= point[0]
        y= point[1]
        if x != None and y != None:
            x = x-size[0]//2
            y = y-size[1]//2
        return x,y

    def close(self,):
        self.camera.close()

    def __init__(self,
            resolution=(2592,1944),
            framerate=30,
            filename = None,
            kSize=(3,3),
            typeKernel = cv2.MORPH_ELLIPSE,
            showImage = False,
            showDetect = False,
            ):
        #camera config
        self.resolution = resolution
        self.framerate = framerate
        self.filename = filename
        self.iso = 1600
        self.speed = 5000
        #position vars
        self.xmean = None
        self.ymean = None
        #solar size
        self.solarSize= None
        #kernel
        self.kernel = cv2.getStructuringElement(typeKernel, kSize)
        #threshold values
        self.thVal = 125
        self.thFill = 255
        self.vflip = False
        self.hflip = False
        #delay time
        self.delay = None
        #file
        if self.filename != None:
            myfile = open(filename,'w')
            myfile.close()
        camera = picamera.PiCamera(
                resolution=self.resolution,
                framerate=self.framerate)
        camera.vflip = self.vflip
        camera.hflip = self.hflip
        self.pyCamConfig(camera, iso=self.iso, sSpeed= self.speed)
        self.camera = camera
        #show images
        self.showImage = showImage
        self.showDetect = showDetect
 
    def detect(self,
           ):
        xmean = self.xmean
        ymean = self.ymean
        #initial date
        date0=datetime.now()
        #while(True):
        with picamera.array.PiRGBArray(self.camera) as stream:
            #tick count
            #e0 = cv2.getTickCount()
            #get image
            self.camera.capture(stream,
                format='bgr',
                use_video_port=True,
                )
            image = stream.array
        #transform
        HSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        #divide
        H,S,V = cv2.split(HSV)
        #Apply threshold to V channel
        ret, thimage = cv2.threshold(
                V,
                self.thVal,
                self.thFill,
                cv2.THRESH_BINARY,
                )
        #get image mean
        tmpX, tmpY, self.solarSize = self.meanXY(thimage)
        #change reference to center
        tmpX, tmpY = self.tocenter((tmpX, tmpY), self.resolution) 
        #end time
        date1=datetime.now()
        if xmean != None and tmpX!=None:
            #draw center
            cv2.line(thimage,
                    (0,self.resolution[1]//2),
                    (self.resolution[0],self.resolution[1]//2),
                    200,
                    )
            cv2.line(thimage,
                    (self.resolution[0]//2,0),
                    (self.resolution[0]//2, self.resolution[1]),
                    200,
                    )
 
            #draw lines
            yline=ymean+self.resolution[1]//2
            xline=xmean+self.resolution[0]//2
            cv2.line(thimage,(0,yline),
                    (self.resolution[0],yline),
                    255,
                    )
            cv2.line(thimage,(xline,0),
                    (xline, self.resolution[1]),
                    255,
                    )
            #calculate distance
            xmean-= tmpX
            ymean-= tmpY
            d = math.sqrt(xmean*xmean+ymean*ymean)
            interval = date1-date0
            speed = d/interval.total_seconds()
            #convert to string
            wStr=str(date1)+','+str(tmpX)+','+str(tmpY)+','+str(speed)
            wStr=wStr+','+str(self.solarSize)+'\n'
        else:
            wStr = str(date1)+',NAN,NAN,NAN,NAN\n'
        #save actual position
        self.xmean = tmpX
        self.ymean = tmpY
        #show
        if self.showImage == True:
            cv2.imshow('image', image)
        if self.showDetect == True:
            cv2.imshow('umbral', thimage)
        #tick counter
        #e2 = cv2.getTickCount()
        #totalT=(e2-e0)/frec
        if self.showImage == True or self.showDetect == True:
            key = cv2.waitKey(1) & 0xFF 
            if key == ord('e'):
                self.thVal+=5
            elif key == ord('w'):
                self.thVal-=5
            elif key == ord('s'):
                self.speed -= 500
            elif key == ord('d'):
                self.speed += 500
            elif key == ord('x'):
                self.iso -= 100
            elif key == ord('c'):
                self.iso += 100
            
            #print parameters
            print('TH:',self.thVal, 'Ss:', self.speed, 'ISO:', self.iso)
        #print data
        print(wStr, end='')
        #write to a file
        if self.filename != None:
            myfile = open(self.filename, 'a')
            myfile.write(wStr)
            myfile.close()
        #delay
        try:
            time.sleep(float(self.delay))
        except:
            pass
        return tmpX, tmpY


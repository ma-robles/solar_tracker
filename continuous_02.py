#coding: utf-8
#Control de motores mediante cámara
#la cámara utiliza capture_continuous

from __future__ import print_function
from __future__ import division
import picamera
import picamera.array
import cv2
import datetime
import time
import stepper
import RPi.GPIO as GPIO
from threading import Thread
import pickle
import os

def calibration(axis1, axis2, steps,camera, loop=1):
    #axis must type class mov
    A=0
    B=0
    C=0
    D=0
    for i in range(loop):
        #get origin
        info,image,thimage = get_center_image(camera)
        center, x0,y0 = info
        #axis1 pos. move
        axis1.mov(steps)
        #get p1
        info,image,thimage = get_center_image(camera)
        center, x,y = info
        d1x = x - x0
        d1y = y - y0
        #axis1 neg. move
        axis1.mov(-steps)
        info,image,thimage = get_center_image(camera)
        center, x,y = info
        d2x = (x - x0)>>1
        d2y = (y - y0)>>1
        A += (d1x-d2x)/steps
        B += (d1y-d2y)/steps
        x0 = x
        y0 = y
        #axis2 pos. move
        axis2.mov(steps)
        info, image,thimage = get_center_image(camera)
        center, x,y = info
        d1x = x - x0
        d1y = y - y0
        #axis2 neg. move
        axis2.mov(-steps)
        info, image,thimage = get_center_image(camera)
        center, x,y = info
        d2x = (x - x0)>>1
        d2y = (y - y0)>>1
        C += (d1x-d2x)/steps
        D += (d1y-d2y)/steps
        print(d1x,d2x,d1y,d2y)
    A=A/loop
    B=B/loop
    C=C/loop
    D=D/loop
    alpha = 1/(D*A-C*B)
    E = -alpha*C
    F = alpha*A
    G = (1+alpha*B*C)/A
    H = -alpha*B
    return E,F,G,H

def getpos(event, x, y, flags, param):
    '''
    get cursor position
    '''
    if event == cv2.EVENT_LBUTTONDBLCLK:
        print(x,y)

def meanXY(image):
    '''
    get image centroid
    '''
    y,x = image.nonzero()
    try:
        return(int(x.mean()), int(y.mean()), len(x))
    except ValueError:
        return(None,None,None)

def drawX(image, pos, resolution):
    '''
    draw position
    '''
    cv2.line(image,
            (0,pos[1]),
            (resolution[0],pos[1]),
            255,
            )
    cv2.line(image,
            (pos[0],0),
            (pos[0],resolution[1]),
            255,
            )

def drawCenter(image, resolution):
    '''
    draw center
    '''
    cv2.circle(image, (resolution[0]>>1,resolution[1]>>1), 15, 127,2)
    pass

class mov():
    '''
    motor movement
    extend with neg_step and pos_step
    '''
    def __init__(self, limit):
        self.limit = limit
    def mov(self, steps):
        if steps > limit:
            steps = limit
        elif (-steps)>limit:
            steps =(-limit)
        #print('steps',steps)
        #print('time2_1:',datetime.datetime.now()-init_time)
        if steps<0:
            for i in range(abs(steps)):
                self.neg_step()
        elif steps>0:
            for i in range(abs(steps)):
                self.pos_step()
class movX(mov):
    def neg_step(self,):
        motor2.dec()
    def pos_step(self,):
        motor2.inc()

class movY(mov):
    def neg_step(self,):
        motor1.dec()
    def pos_step(self,):
        motor1.inc()
def dsun():
    global key
    global pos
    while key!='q':
        print(pos)


def config(camera):
    global iso
    global shutter_speed
    global vflip
    global hflip
    global exposure_mode
    global awb_mode
    #picamera config
    #camera = picamera.PiCamera(
    #        resolution=resolution,
    #        framerate = framerate,
    #        )
    camera.vflip = vflip
    camera.hflip = hflip
    camera.iso = iso
    camera.exposure_mode = exposure_mode
    camera.awb_mode = awb_mode
    camera.shutter_speed = shutter_speed

def get_center_image(camera):
    '''
    image capture and get center
    '''
    with picamera.array.PiRGBArray(camera) as stream:
        camera.capture(
                stream,
                format='bgr',
                use_video_port = True,
                )
        image = stream.array
    HSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    H,S,V = cv2.split(HSV)
    ret, thimage = cv2.threshold(
            V,
            thVal,
            255,
            cv2.THRESH_BINARY,
            )
    #thimage = cv2.morphologyEx(
    #        thimage,
    #        cv2.MORPH_OPEN,
    #        kernel,
    #        )
    #get center
    return meanXY(thimage), image, thimage

def camera_detect(no_move=False): 
    global camera
    global thVal
    
    th_px = 0
    movstep = 1 
    #movement ctrl
    flag_axis1 = True
    window= 'DISPLAY' in os.environ
    print(window)
    if window == True:
        cv2.namedWindow('image')
        cv2.startWindowThread()
    #loop
    with picamera.array.PiRGBArray(camera) as stream:
        for frame in camera.capture_continuous(
            stream,
            format='bgr',
            use_video_port = True,
            ):
            init_time = datetime.datetime.now()
            image = frame.array
            HSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            H,S,V = cv2.split(HSV)
            ret, thimage = cv2.threshold(
                    V,
                    thVal,
                    255,
                    cv2.THRESH_BINARY,
                    )
            #thimage = cv2.morphologyEx(
            #        thimage,
            #        cv2.MORPH_OPEN,
            #        kernel,
            #        )
            #get center
            tmpX, tmpY, solarSize = meanXY(thimage)
            if solarSize < 50:
                tmpX = None
                tmpY = None
            if tmpX != None and tmpY != None and no_move==False:
                drawX(thimage, (tmpX,tmpY), resolution)
                #translate
                #offset
                tmpX = tmpX-365#(resolution[0]>>1)+20#//2
                tmpY = tmpY-190#(resolution[1]>>1)+100#//2
                if flag_axis1 == True:
                    flag_axis1 = False
                    #mov_axis1 = tmpX*multX1+tmpY*multX2 
                    if tmpX<(-th_px):
                        movX1.mov(-movstep)
                    elif tmpX>th_px:
                        movX1.mov(movstep)
                else:
                    flag_axis1 = True
                    #mov_axis2 = tmpX*multY1+tmpY*multY2 
                    if tmpY<(-th_px):
                        movY1.mov(5*movstep)
                    elif tmpY>th_px:
                        movY1.mov(-5*movstep)
            if window== True:
                cv2.imshow('image', thimage)
                cv2.waitKey(1)
            #stream.seek(0)
            stream.truncate(0)
            print (
                    tmpX,
                    tmpY,
                    #multX1,
                    #multY2,
                    int(1000*(datetime.datetime.now()-init_time).total_seconds()),
                    )
            if key == 'q':
                break
    if window == True:
        cv2.destroyWindow('image')
        cv2.waitKey(1)


def save_vars(fileobj, *args):
    for arg in args:
        pickle.dump(arg,fileobj)

def save_conf():
    global thVal
    global resolution
    global framerate
    global iso
    global vflip
    global hflip
    global exposure_mode
    global awb_mode
    global shutter_speed
    global dstepx
    global dstepy
    global posx
    global posy

    with open('config.cca','wb') as myfile:
        save_vars(myfile,
                thVal,
                resolution,
                framerate,
                iso,
                vflip,
                hflip,
                exposure_mode,
                awb_mode,
                shutter_speed,
                dstepx,
                dstepy,
                posx,
                posy,
                )

def load_conf():
    global thVal
    global resolution
    global framerate
    global iso
    global vflip
    global hflip
    global exposure_mode
    global awb_mode
    global shutter_speed
    global dstepx
    global dstepy
    global posx
    global posy

    with open('config.cca','rb') as myfile:
        thVal = pickle.load(myfile)
        resolution = pickle.load(myfile)
        framerate = pickle.load(myfile)
        iso = pickle.load(myfile)
        vflip = pickle.load(myfile)
        hflip = pickle.load(myfile)
        exposure_mode = pickle.load(myfile)
        awb_mode = pickle.load(myfile)
        shutter_speed = pickle.load(myfile)
        dstepx = pickle.load(myfile)
        dstepy = pickle.load(myfile)
        posx = pickle.load(myfile)
        posy = pickle.load(myfile)

##define motor pins
print('init input/output')
dirMotor1 = 4
speed1 = 17
dirMotor2 = 27
speed2 = 22
GPIO.setmode(GPIO.BCM)
pulse_time=0.001
#config outputs
GPIO.setup(dirMotor1, GPIO.OUT)
GPIO.setup(dirMotor2, GPIO.OUT)
GPIO.setup(speed1, GPIO.OUT)
GPIO.setup(speed2, GPIO.OUT)
#dir
GPIO.output(dirMotor1, GPIO.LOW)
GPIO.output(dirMotor2, GPIO.HIGH)
#pwm
motorPWM1 = GPIO.PWM(speed1,100)
motorPWM2 = GPIO.PWM(speed2,100)
#duty cycle
motorPWM1.start(50)
motorPWM2.start(50)
#define axis motor
motor1 = stepper.ctr_motor(speed1, dirMotor1, pulse_time=pulse_time)
motor2 = stepper.ctr_motor(speed2, dirMotor2, pulse_time=pulse_time)
multX1 = 0.6
multX2 = 0.0
multY1 = 0.0
multY2 = 0.4
limit = 10000
movX1 = movX(limit)
movY1 = movY(limit)

#loading init values
try:
    load_conf()
    print('successful load')
    
except (IOError,EOFError):
    print('config file not found or corrupted')
    print('loading default values')
    thVal = 150
    #camera init values
    resolution = (640,480)
    framerate = 19
    iso = 100
    vflip = True 
    hflip = False
    exposure_mode = 'off'
    awb_mode = 'off'
    shutter_speed = 1500
    #motor init values
    dstepx=0.18
    dstepy=0.18
    posx = 0
    posy = 0
    #save
    print('creating config file')
    save_conf()
#init camera
#640x480
camera = picamera.PiCamera(resolution=(800,600),framerate=19)
#config camera
config(camera)
#filter settings
kSize=(3,3)
typeKernel = cv2.MORPH_ELLIPSE
kernel = cv2.getStructuringElement(typeKernel, kSize)
#command interpreter
key = 0
print('Command interpreter started')
while key!='exit':
    key = raw_input('cmd:')
    if key =='':
        continue
    cmd = key.split()
    if cmd[0] == 'vflip':
        if cmd[1]=='true':
            vflip = True
        if cmd[1]=='false':
            vflip = False
    elif cmd[0] == 'hflip':
        if cmd[1]=='true':
            hflip = True
        if cmd[1] =='false':
            hflip = False
    
    elif cmd[0] == 'show':
        if cmd[1] == 'conf':
            with open('config.cca','rb') as myfile:
                counter = 0
                while True:
                    try:
                        print(counter,pickle.load(myfile))
                        counter+=1
                    except EOFError:
                        break

        if cmd[1] == 'pos':
            for name,val in pos.items():
                print(name,':', val)
    elif cmd[0] == 'set':
        if cmd[1] == 'home':
            pos['X'] = 0
            pos['Y'] = 0
    elif cmd[0] == 'dstepx':
        dstepx=float(cmd[1])
    elif cmd[0] == 'dstepy':
        dstepy=float(cmd[1])
    
    elif cmd[0] == 'mvx':
        try:
            dist = int(cmd[1])
            movX1.mov(dist)
            pos['X'] += dist
        except:
            print('error')
            
    elif cmd[0] == 'mvy':
        try:
            dist = int(cmd[1])
            movY1.mov(dist)
            pos['Y']+=dist
        except:
            print('error')
    elif cmd[0] == 'home':
        dist = -(pos['X'])
        print('dx', dist)
        movX1.mov(dist)
        dist = -(pos['Y'])
        print('dy', dist)
        movY1.mov(dist)
        pos['X'] = 0
        pos['Y'] = 0

    elif cmd[0] == 'start':
        thr = Thread(target=camera_detect)
        thr.daemon = True
        thr.start()

    elif cmd[0] == 'preview':
        thr = Thread(target=camera_detect, args=(True,))
        thr.daemon = True
        thr.start()

    elif cmd[0] == 'config':
        #if not 'camera' in locals():
        #camera = picamera.PiCamera(resolution=resolution, framerate=framerate)
        #config(camera)
        #window
        def callback(pos):
            global thVal
            thVal = pos    
        cv2.namedWindow('image')
        cv2.startWindowThread()
        def calliso(iso):
            global camera
            camera.iso=iso

        def callshut(shutter):
            global camera
            camera.shutter_speed=shutter
        #trackbars
        cv2.createTrackbar('th', 'image', thVal,255,callback )
        cv2.createTrackbar('iso', 'image', camera.iso,1600,calliso )
        cv2.createTrackbar('shut', 'image', camera.shutter_speed,20000,callshut)
        while True:
            info,image,thimage = get_center_image(camera)
            cv2.imshow('image', thimage)
            wk= cv2.waitKey(500)&0xff
            if wk == 113:
                #'q' exit
                break
            elif wk ==115:
                #'s' save
                save_conf()
                print('config saved')

        print(camera.iso,camera.shutter_speed,thVal)
        #cv2.waitKey(1)
        cv2.destroyWindow('image')
        cv2.waitKey(1)
    elif cmd[0]=='save':
        #save configuration
        save_conf()
    elif cmd[0]=='load':
        #load configuration
        load_conf()
    else:
        print('command not found')
 
GPIO.cleanup()
camera.close()

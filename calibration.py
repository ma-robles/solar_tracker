from __future__ import division
import solarDtr as solar
import stepper
import RPi.GPIO as GPIO
from threading import Thread
from time import sleep
import solarMov

class LoopX(solarMov.Loop):
    def neg_step(self,):
        motor2.dec()
    def pos_step(self,):
        motor2.inc()
class LoopY(solarMov.Loop):
    def neg_step(self,):
        motor1.inc()
    def pos_step(self,):
        motor1.dec()

def sal():
    global key
    while(key!='q'):
        key=raw_input()

def detect():
    global x
    global y
    global resolution
    global wait_time
    #sun = solar.detector(filename='test/test02.cca')
    sun = solar.detector(resolution = resolution,
            showDetect=True,
            #showImage=True,
            )
    #sun.delay=0.3
    while(True):
        x,y=sun.detect()
        sun.delay=wait_time+0.01
 
#global vars
x=0
y=0
resolution = (640,480)
wait_time =0
key=''
##define motor pins
dirMotor1 = 4
speed1 = 17
dirMotor2 = 27
speed2 = 22
GPIO.setmode(GPIO.BCM)
motor1 = stepper.ctr_motor(speed1, dirMotor1)
motor2 = stepper.ctr_motor(speed2, dirMotor2)
max_vel=10
movX = LoopX(1,2000)
movY = LoopY(1,2000)
movX.start()
movY.start()
thX =1# resolution[0]//200
thY =1# resolution[1]//200
#launch key detect
t = Thread(target=sal)
t.daemon=True
t.start()
#launch sun detector
#thread_s=Thread(target=detect)
#thread_s.daemon = True
#thread_s.start()
sun = solar.detector(
        resolution = resolution,
        framerate = 19,
        showDetect=True,
        showImage=True,
        )
mymov=30
while (key!='q'):
    sun.detect()
    if key=='a':
        mymov=mymov//2
        print mymov
    if key=='s':
        mymov=mymov*2
        print mymov
    if key == 'x':
        movX.put(mymov)
        movY.put(mymov)
    if key == 'z':
        movX.put(-mymov)
        movY.put(-mymov)
    if key!='q':
        key = ''
movX.condition = False
movY.condition = False
GPIO.cleanup()
def calibration(axis1, axis2, image, steps):
    #axis must type class mov
    dx = 0
    dy = 0
    wait = 2
    center,xi,yi = meanXY(image)

    axis1.mov(steps)
    time.sleep(wait)
    center,xf,yf = meanXY(image)
    d1x = xf - xi
    d1y = yf - yi
    print('x,y',xi,xf,yi,yf,d1x,d1y)
    xi = xf
    yi = yf
    axis1.mov(-steps)
    time.sleep(wait)
    center, xf,yf = meanXY(image)
    d2x = xf - xi
    d2y = yf - yi
    A = (d2x/2-d1x)/steps
    B = (d2y/2-d1y)/steps
    print('x,y',xi,xf,yi,yf,d1x,d1y)
    print(d1x,d2x,d1y,d2y)

    axis2.mov(steps)
    time.sleep(wait)
    center,xf,yf = meanXY(image)
    d1x = xf - xi
    d1y = yf - yi
    xi = xf
    yi = yf
    axis2.mov(-steps)
    time.sleep(wait)
    center, xf,yf = meanXY(image)
    d2x = xf - xi
    d2y = yf - yi
    C = (d2x/2-d1x)/steps
    D = (d2y/2-d1y)/steps
    print(d1x,d2x,d1y,d2y)
    
    alfa = 1/(D*A-C*B)
    E = -alpha*C
    F = alpha*A
    G = (1+alpha*B*C)/A
    H = -alpha*B
    return E,F,G,H


#coding: utf-8
#Control de motores mediante cámara
#la cmara utiliza capture_continuous

from __future__ import print_function
from __future__ import division
import cv2
import datetime
import time
import stepper
import RPi.GPIO as GPIO

def sal():
    global key
    while key!='q':
        key = raw_input()

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
##define motor pins
dirMotor1 = 4
speed1 = 17
dirMotor2 = 27
speed2 = 22
GPIO.setmode(GPIO.BCM)
#pulse_time=0.001
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
motorPWM1 = GPIO.PWM(speed1,1)
motorPWM2 = GPIO.PWM(speed2,1)
#duty cycle
#motorPWM1.start(50)
#motorPWM2.start(50)
motor1 = stepper.ctr_motor(speed1, dirMotor1, pulse_time=pulse_time)
motor2 = stepper.ctr_motor(speed2, dirMotor2, pulse_time=pulse_time)
multX1 = 0.6
multX2 = 0.0
multY1 = 0.0
multY2 = 0.4
limit = 500000
movX1 = movX(limit)
movY1 = movY(limit)


#init values
resolution = (640,480)
kSize=(3,3)
typeKernel = cv2.MORPH_ELLIPSE
kernel = cv2.getStructuringElement(typeKernel, kSize)
#threshold value
thVal = 150
#picamera config
#key control
key = 0
steps = 100
#movement ctrl
flag_axis1 = True

while(key!='q'):
    tmpX = 0
    tmpY = 0
    #movstep =207080 
    movstep =50 
    movstep_fine = 1 
    #get cmd
    key = raw_input()

    if key == 'a':
        tmpX = -movstep
    elif key == 's':
        tmpX =-movstep_fine
    elif key == 'd':
        tmpX = movstep_fine
    elif key == 'f':
        tmpX = movstep
    elif key == 'q':
        break
    elif key == 'w':
        tmpY = -movstep
    elif key == 'e':
        tmpY = -movstep_fine
    elif key == 'r':
        tmpY = movstep
    elif key == 't':
        tmpY = movstep
    elif key == 'p':
        print(datetime.datetime.now())
    elif key =='c':
        #coef=calibration(movX1,
        #    movY1,
        #    30,
        #    camera,
        #    5,
        #    )
        #mutlX1,multX2,multY1,multY2 = coef
        #print('calibración:',coef)
        pass
    elif key == 'n':
        movstep= int(raw_input('movstep:'))
        movstep_fine = int(raw_input('fine:'))
    key =0
    #movement
    movX1.mov(tmpX)
    movY1.mov(tmpY)
key=0
mov_step=0
#loop
GPIO.cleanup()


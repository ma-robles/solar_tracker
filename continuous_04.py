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
import solarPos
import math
import pwm_lib
import numpy as np

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
class movAz(mov):
    def neg_step(self,):
        motor_az.dec()
    def pos_step(self,):
        motor_az.inc()

class movEl(mov):
    def neg_step(self,):
        motor_el.dec()
    def pos_step(self,):
        motor_el.inc()
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
    tmpX,tmpY,solarSize=meanXY(thimage) 
    if solarSize < 50:
        tmpX = None
        tmpY = None
        ret, thimage = cv2.threshold(
                V,
                15,#thval
                255,
                cv2.THRESH_BINARY,
                )
    elif solarSize>100:
        ret, thimage = cv2.threshold(
                V,
                thVal,#thval
                255,
                cv2.THRESH_BINARY,
                )
    tmpX, tmpY, solarSize = meanXY(thimage)
    #print('Solar size:',solarSize)
    if solarSize < 50:
        tmpX = None
        tmpY = None
    return (tmpX, tmpY, solarSize), image, thimage

def update_dir(speed_time, mult_az=1, mult_el=1):
    global speed_table_el
    global speed_table_az
    global el_addr
    global az_addr

    speed_el=speed_table_el[speed_time]*mult_el
    speed_az=speed_table_az[speed_time]*mult_az
    if speed_el>=0:
        #GPIO.output(dirMotor_el, GPIO.LOW)
        pwm_lib.send(el_addr, 4, [0,2])
    else:
        #GPIO.output(dirMotor_el, GPIO.HIGH)
        pwm_lib.send(el_addr, 4, [0,1])
    #print('test:',speed_el, pwm_lib.pack(pwm_lib.val(abs(speed_el))))
    pwm_lib.send(el_addr, 4, pwm_lib.pack(pwm_lib.val(abs(speed_el))))

    if speed_az>=0:
        #GPIO.output(dirMotor_az, GPIO.LOW)
        pwm_lib.send(az_addr, 4, [0,2])
    else:
        #GPIO.output(dirMotor_az, GPIO.HIGH)
        pwm_lib.send(az_addr, 4, [0,1])
    pwm_lib.send(az_addr, 4, pwm_lib.pack(pwm_lib.val(abs(speed_az))))

    return speed_az, speed_el
def get_posfile():
    with open('mypos.cca') as posfile:
        az = pickle.load(posfile)
        el = pickle.load(posfile)
        return az, el
            
def move_deg(move_d, dstep, addr):

    dist = float(move_d)//(dstep)
    pwm_lib.send(addr,2,[0])
    pwm_lib.send(addr, 1, pwm_lib.pack4(dist))
    busy = True
    while (busy==True):
        try:
            time.sleep(0.5)
            pwm_lib.send(addr,2,[1])
            busy = False
            print('listo!')
        except IOError:
            pass

def camera_detect(no_move=False): 
    global camera
    global thVal
    global offset
    global posx
    global posy

    global speed_table_az
    global speed_table_el
    global az_addr
    global el_addr
    
    #start movement if apply
    if no_move==False:
        pwm_active=True
        speed_time= datetime.datetime.now()
        speed_time=datetime.datetime(
                speed_time.year,
                speed_time.month,
                speed_time.day,
                speed_time.hour,
                speed_time.minute,
                )
        #FIX direction
        #dir
        vel_az,vel_el=update_dir(speed_time)
        print('vel_el:',vel_el,'vel_az:',vel_az)
    print("offset:",offset)
    th_px =1.0 
    #movement ctrl
    window= 'DISPLAY' in os.environ
    print(window)
    if window == True:
        cv2.namedWindow('image')
        cv2.startWindowThread()
    lap_delay_x=0
    lap_delay_y=0
    multx=1.0
    multy=1.0
    tmp_x0=0
    tmp_y0=0
    vel_y=0
    vel_x=0
    counter=0
    filename=datetime.datetime.now().strftime("%Y%m%d_%H%M")+'.log'
    print(filename)
    fp=open(filename,'w')
    fp.close()
    ang_file = open('angles.cca','w')
    ang_file.close()
    angle_x = 0
    #loop
    while True:
        init_time = datetime.datetime.now()
        (tmpX, tmpY, solarSize), image, thimage=get_center_image(camera)
        o_pos=()
        
        if no_move==False:
            speed_time2=datetime.datetime(
                    init_time.year,
                    init_time.month,
                    init_time.day,
                    init_time.hour,
                    init_time.minute,
                    )
            az1,el1 = get_solar(speed_time2)
            #save position
            with open('mypos.cca', 'w') as posfile:
                pickle.dump( az1, posfile)
                pickle.dump( el1, posfile)
            taz,tel = get_posfile()
            
            #FIX test PWM control
            #FIX change direction
            if speed_time<speed_time2:
                ##testing axis
                #if tmpX!= None:
                #    #stop
                #    pwm_lib.send(az_addr, 4, [0,0])
                #    pwm_lib.send(el_addr, 4, [0,0])
                #    #move
                #    move_d = 0.6
                #    #move x
                #    move_deg(move_d, dstep_az*16, az_addr)
                #    (tmpX2, tmpY2, solarSize), image, thimage=get_center_image(camera)
                #    tmpX2-=tmpX
                #    tmpY2-=tmpY
                #    angle_x = np.angle(tmpX2+tmpY2*1j, deg=True)
                #    with open('angles.cca', 'a') as ang_file:
                #        print(az1, el1, tmpX2, tmpY2, angle_x, end=' ', file = ang_file)
                #    move_deg(-move_d, dstep_az*16, az_addr)
                #    (tmpX, tmpY, solarSize), image, thimage=get_center_image(camera)
                #    #move y
                #    move_deg(move_d/2, dstep_el, el_addr)
                #    (tmpX2, tmpY2, solarSize), image, thimage=get_center_image(camera)
                #    tmpX2-=tmpX
                #    tmpY2-=tmpY
                #    move_deg(-move_d/2, dstep_el, el_addr)
                #    angle_y = np.angle(tmpX2+tmpY2*1j, deg=True)
                #    with open('angles.cca', 'a') as ang_file:
                #        print(tmpX2, tmpY2, angle_y, file= ang_file)
                #    print(angle_x, angle_y)
                #    #get new pos
                #    (tmpX, tmpY, solarSize), image, thimage=get_center_image(camera)
                #    #end test
                vel_az, vel_el=update_dir(speed_time2,multx,multy)
                print(speed_time2,
                        speed_table_el[speed_time2],
                        speed_table_az[speed_time2],
                        )

                speed_time=speed_time2
            if tmpX == None and tmpY == None:
                multx=1.0
                multy=1.0
                print('vAz:',vel_az,
                        'vEl:',vel_el,
                        'mul:',multx,multy)

            else:
                drawX(thimage, (tmpX,tmpY), resolution)
                #translate
                #apply offset
                tmpX = tmpX-offset[0]
                tmpY = tmpY-offset[1]
                o_pos = (tmpX, tmpY)
                #print('original:',tmpX,tmpY)
                print(
                        #speed_time2,
                        'az:', np.round(az1),
                        'el:', np.round(el1),
                        'dif:', np.round(az1-el1),
                        end=' '

                        )

                #tmpX,tmpY=trans2( np.radians(angle_x),tmpX,tmpY)
                tmpX,tmpY=trans(az1, el1, tmpX, tmpY)
                vel_x=abs(tmpX)-abs(tmp_x0)
                vel_y=abs(tmpY)-abs(tmp_y0)
                tmp_x0=tmpX
                tmp_y0 = tmpY
                #control
                ######################################################
                #min_speed = 0.2
                #max_speed = 3.5
                min_speed = 0.0
                max_speed = 2.0
                #original
                #step_speed = 0.005
                step_speed_y = 0.050
                step_speed = 0.050
                if speed_table_el[speed_time2]>0:
                    tmpY=-tmpY
                if speed_table_az[speed_time2]<0:
                    tmpX=-tmpX
                #if el1<80:
                if tmpX<-th_px and vel_x>=0:
                    multx += step_speed#*abs(tmpX)
                    if multx>max_speed:
                        multx=max_speed
                    vel_az=speed_table_az[speed_time2]*multx
                    vel_data=pwm_lib.val(abs(vel_az))
                    pwm_lib.send(az_addr, 4, pwm_lib.pack(vel_data))
                if tmpX>th_px and vel_x>=0:
                    multx -= step_speed#*abs(tmpX)
                    if multx<min_speed:
                        multx=min_speed
                    vel_az=speed_table_az[speed_time2]*multx
                    vel_data=pwm_lib.val(abs(vel_az))
                    pwm_lib.send(az_addr, 4, pwm_lib.pack(vel_data))
                #if tmpX>-0.5*th_px and tmpX<0.5*th_px:
                #    multx=(1+multx)/2

                if tmpY>th_px and vel_y>=0:
                
                    multy -= step_speed_y#*abs(tmpY)
                    if multy <min_speed:
                        multy=min_speed
                    vel_el=speed_table_el[speed_time2]*multy
                    vel_data=pwm_lib.val(abs(vel_el))
                    #print('test1:', vel_data, pwm_lib.pack(vel_data))
                    pwm_lib.send(el_addr, 4, pwm_lib.pack(vel_data))
                if tmpY<-th_px and vel_y>=0:
                    multy += step_speed_y#*abs(tmpY)
                    if multy >max_speed:
                        multy=max_speed
                    vel_el=speed_table_el[speed_time2]*multy
                    vel_data=pwm_lib.val(abs(vel_el))
                    pwm_lib.send(el_addr, 4, pwm_lib.pack(vel_data))
                #if tmpY>-0.5*th_px and tmpY<0.5*th_px:
                #    multy=(1+multy)/2
                print('vAz:', np.round(vel_az, decimals=1),
                        'vEl:',np.round(vel_el, decimals=1),
                        'mul:',
                        np.round(multx, decimals=2),
                        np.round(multy, decimals=2),
                        'Sol:', np.round(solarSize),
                        end=' ',
                        )
                #log
                counter+=1
                if counter==5:
                    counter=0
                    with open(filename,'a') as fp:
                        print(
                                speed_time2,
                                vel_az,vel_el,
                                tmpX, tmpY,
                                multx,multy,solarSize,file=fp)
                #if tmpX==0 and tmpY==0:
                #    multx=1
                #    multy=1
                #    vel_az=speed_table_az[speed_time2]*multx
                #    vel_data=pwm_lib.val(abs(vel_az))
                #    pwm_lib.send(az_addr, 4, pwm_lib.pack(vel_data))
                #    vel_el=speed_table_el[speed_time2]*multy
                #    vel_data=pwm_lib.val(abs(vel_el))
                #    pwm_lib.send(el_addr, 4, pwm_lib.pack(vel_data))

        if window== True:
            cv2.imshow('image', thimage)
            cv2.waitKey(1)
        if tmpX!=None:
            print (
                'pos:',
                o_pos,
                np.round(tmpX, decimals=1),
                np.round(tmpY, decimals=1),
                'time:',
                int(1000*(datetime.datetime.now()-init_time).total_seconds()),
                )
        if key == 'q':
            pwm_lib.send(az_addr, 4, [0,0])
            pwm_lib.send(el_addr, 4, [0,0])
            break
        time.sleep(3)
    if window == True:
        cv2.destroyWindow('image')
        cv2.waitKey(1)
    pwm_active=False


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
    global dstep_el
    global dstep_az
    global posx
    global posy
    global offset

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
                dstep_el,
                dstep_az,
                posx,
                posy,
                offset,
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
    global dstep_el
    global dstep_az
    global posx
    global posy
    global offset

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
        dstep_el = pickle.load(myfile)
        dstep_az = pickle.load(myfile)
        posx = pickle.load(myfile)
        posy = pickle.load(myfile)
        offset = pickle.load(myfile)

def derivate(values):
    i=1
    der=[]
    while i<len(values):
        der.append(values[i]-values[i-1])
        i+=1
    return der

def calculate_pos(mydate,k_az=1,k_el=1):
    #k_el*=0.5
    mydate =datetime.datetime(mydate.year, mydate.month, mydate.day)
    track_time = mydate+datetime.timedelta(hours=8,)
    end_time = mydate+datetime.timedelta(hours=19,)
    inter = datetime.timedelta(minutes=1)

    mytime = []
    azimuth = []
    elevation = [] 
    while(track_time<end_time):
        sol=solarPos.solarPos(19.4, -99.149999, track_time,-6)
        mytime.append(track_time)
        azimuth.append(math.degrees(sol.az))
        elevation.append(math.degrees(sol.el))
        track_time+=inter
    vel_az=derivate(azimuth)
    flag_change=False
    for i in range(len(vel_az)):
        if vel_az[i]>180:
            flag_change =True
        if flag_change==True:
            azimuth[i+1]-=360
    vel_az=derivate(azimuth)
    vel_el=derivate(elevation)
    table_az={}
    for i,val in enumerate(vel_az):
        table_az[mytime[i]]=val*k_az
    table_el={}
    for i,val in enumerate(vel_el):
        table_el[mytime[i]]=val*k_el
    return table_az, table_el
        
#def print_pos(mydate):
def get_solar(mydate):
    sol = solarPos.solarPos(19.4,-99.149999,mydate,-6)
    az=math.degrees(sol.az)
    el=math.degrees(sol.el)
    #print(mydate,'azimuth=', posy, 'elevation:',posx)
    return az,el

def trans2(angle, posx, posy):
    x = posx*math.cos(-angle)-posy*math.sin(-angle)
    y = posx*math.sin(-angle)+posy*math.cos(-angle)
    return x,y


def trans(azimuth, elevation,posx,posy):
    #horario
    angle = -90+azimuth-elevation
    angle = np.radians(angle)
    x=posx*math.cos(angle)-posy*math.sin(angle)
    #check
    #angle = -90+np.sin(np.radians(90-elevation))*(azimuth-elevation)
    #angle = np.radians(angle)
    y=posx*math.sin(angle)+posy*math.cos(angle)

    return x,y
    
##define motor pins
print('init input/output')
GPIO.setmode(GPIO.BCM)
#stop
#az
az_addr= 0x11
pwm_lib.send(az_addr, 4, [0,0])
#el
el_addr= 0x12
pwm_lib.send(el_addr, 4, [0,0])

#loading init values
resolution=(800,600)
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
    dstep_el=0.18
    dstep_az=0.18
    posx = 0
    posy = 0
    offset=[resolution[0]>>1,resolution[1]>>1]
    #save
    print('creating config file')
    save_conf()
#motor resolution
#entre 0.29-0.28
#0.235
dstep_el =1*0.0144/50
#original 0.92
dstep_az = 0.99*(0.001761/16)
print('deg/step:', dstep_el,dstep_az)
print(1/dstep_el*60,1/dstep_az*60)
#speed tables
#f=V/(DS*60)
#f - frequency
#V - degrees by min
#DS - degrees by step
speed_table_az,speed_table_el=calculate_pos(datetime.datetime.now(),
        1/(dstep_az*60),
        1/(dstep_el*60),
        )

test_time=datetime.datetime.now()
test_time=datetime.datetime(test_time.year,
        test_time.month,
        test_time.day,
        test_time.hour,
        test_time.minute,
        )
#test_time2=datetime.datetime(2015,9,28,12)
print('date:',test_time) 
print('az_table:',len(speed_table_az))
print(speed_table_az[test_time])
#print(speed_table_az[test_time2])
print('el_table:',len(speed_table_el))
print(speed_table_el[test_time])
print(speed_table_el[test_time])
#print(speed_table_el[test_time2])
#init camera
#640x480
framerate=19
camera = picamera.PiCamera(
        resolution=resolution,
        framerate=framerate,
        )
#config camera
config(camera)
#filter settings
kSize=(3,3)
typeKernel = cv2.MORPH_ELLIPSE
kernel = cv2.getStructuringElement(typeKernel, kSize)
#command interpreter
key = 0
#offset point
#offset=[resolution[0]>>1,resolution[1]>>1]
print('Initial offset point:',offset)
try:
    az, el = get_posfile()
except:
    print('load file pos ERROR\n values to zero!')
    az, el = 0.0,0.0
az1,el1 = get_solar(test_time)
print('saved coords:', az, el)
print('new coords:', az1, el1)

key = raw_input('auto pos? y/n:')
if key == 'y':
    print('elevation move...')
    dmov = el1-el
    dist = float(dmov)//dstep_el
    print('d:', dmov, dist)
    pwm_lib.send(el_addr, 2, [0])
    pwm_lib.send(el_addr, 1, pwm_lib.pack4(dist))
    print('azimuth move...')
    dmov = az1-az
    dist = float(dmov)//(dstep_az*16)
    print('d:', dmov, dist)
    pwm_lib.send(az_addr,2,[0])
    pwm_lib.send(az_addr, 1, pwm_lib.pack4(dist))
    busy = True
    while (busy==True):
        try:
            time.sleep(0.5)
            pwm_lib.send(az_addr,2,[1])
            busy = False
            print('azimuth OK')
        except IOError:
            pass
    pwm_lib.send(az_addr, 2, [1])
    busy = True
    while (busy==True):
        try:
            time.sleep(0.5)
            pwm_lib.send(el_addr, 2, [1])
            busy = False
            print('elevation OK')
        except IOError:
            pass
    pwm_lib.send(el_addr, 2, [1])

    test_time=datetime.datetime.now()
    test_time=datetime.datetime(test_time.year,
        test_time.month,
        test_time.day,
        test_time.hour,
        test_time.minute,
        )
    az, el = az1, el1
    az1,el1 = get_solar(test_time)
    print('elevation move...')
    dmov = el1-el
    dist = float(dmov)//dstep_el
    print('d:', dmov, dist)
    pwm_lib.send(el_addr, 2, [0])
    pwm_lib.send(el_addr, 1, pwm_lib.pack4(dist))
    print('azimuth move...')
    dmov = az1-az
    dist = float(dmov)//(dstep_az*16)
    print('d:', dmov, dist)
    pwm_lib.send(az_addr,2,[0])
    pwm_lib.send(az_addr, 1, pwm_lib.pack4(dist))
    busy = True
    while (busy==True):
        try:
            time.sleep(0.5)
            pwm_lib.send(az_addr,2,[1])
            busy = False
            print('azimuth OK')
        except IOError:
            pass
    pwm_lib.send(az_addr, 2, [1])
    busy = True
    while (busy==True):
        try:
            time.sleep(0.5)
            pwm_lib.send(el_addr, 2, [1])
            busy = False
            print('elevation OK')
        except IOError:
            pass
    pwm_lib.send(el_addr, 2, [1])

    #save position
    with open('mypos.cca', 'w') as posfile:
        pickle.dump( az1, posfile)
        pickle.dump( el1, posfile)
else:
    print('No auto pos...')

print('Command interpreter started')
print(datetime.datetime.now())
pwm_active=False
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
    elif cmd[0] == 'dstep_el':
        dstep_el=float(cmd[1])
    elif cmd[0] == 'dstep_az':
        dstep_az=float(cmd[1])
    elif cmd[0]=='j':
        dstep_az-=0.00005
    elif cmd[0]=='k':
        dstep_az+=0.00005
    
    elif cmd[0] == 'mvx':
        try:
            d= float(cmd[1])
            move_deg(d, dstep_az*16, az_addr)
            az, el = get_posfile()
            az += d
            with open('mypos.cca', 'w') as posfile:
                pickle.dump( az, posfile)
                pickle.dump( el, posfile)
        except:
            print('step error')
            
    elif cmd[0] == 'mvy':
        try:
            d= float(cmd[1])
            move_deg(d, dstep_el, el_addr)
            az, el = get_posfile()
            el += d
            with open('mypos.cca', 'w') as posfile:
                pickle.dump( az, posfile)
                pickle.dump( el, posfile)
        except:
            print('step error')

    elif cmd[0] == 'home':
        #FIX
        if pwm_active==True:
            print('Warning:PWM active')
            continue
        #dist = -(pos['X'])
        print('dx', dist)
        #movAz1.mov(dist)
        dist = -(pos['Y'])
        print('dy', dist)
        #movEl1.mov(dist)
        pos['X'] = 0
        pos['Y'] = 0

    elif cmd[0] == 'start':
        pwm_lib.send(az_addr,2,[1])
        
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
                #pwm_lib.send(0)
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
    elif cmd[0]=='get_solar':
        mydate = datetime.datetime.now()
        sol  = solarPos.solarPos(19.4, -99.149999, mydate, -6)
        print('az:', math.degrees(sol.az),
            'el:', math.degrees(sol.el),
            )
    elif cmd[0]=='set_pos':
        mydate=datetime.datetime.now()
        sol = solarPos.solarPos(19.4,-99.149999,mydate,-6)
        posy=math.degrees(sol.az)
        posx=math.degrees(sol.el)
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
        tmpX,tmpY,solarSize=meanXY(thimage) 
        if tmpX == None:
            print('Error no signal')
        else:
            offset = [tmpX,tmpY]
        print('x:',tmpX,'y:',tmpY,'solar:',solarSize)
        print(mydate,'azimuth=', posy, 'elevation:',posx)
    else:
        print('command not found')
 
camera.close()
#stop
pwm_lib.send(az_addr, 4, [0,0])
pwm_lib.send(el_addr, 4, [0,0])

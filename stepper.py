import RPi.GPIO as GPIO
import time
class ctr_motor():
    def __init__(self, speed, dirMotor, pulse_time=0.005):
        self.speed= speed
        self.dirMotor= dirMotor
        self.pulse_time = pulse_time
        GPIO.setup(dirMotor, GPIO.OUT)
        GPIO.setup(speed, GPIO.OUT)
        GPIO.output(dirMotor, GPIO.HIGH)
        GPIO.output(speed, GPIO.LOW)


    def pulse(self):
        GPIO.output(self.speed, GPIO.HIGH)
        time.sleep(self.pulse_time)
        GPIO.output(self.speed, GPIO.LOW)
        time.sleep(self.pulse_time)

    def inc(self):
        GPIO.output(self.dirMotor, GPIO.HIGH)
        self.pulse()

    def dec(self):
        GPIO.output(self.dirMotor, GPIO.LOW)
        self.pulse()

if __name__ =='__main__':
    dirMotor1=4
    speed1 = 17
    dirMotor2= 27
    speed2 = 22
    wait = .05 
    #GPIO.cleanup()
    GPIO.setmode(GPIO.BCM)
    motor1=ctr_motor(speed1, dirMotor1)
    motor2=ctr_motor(speed2, dirMotor2)
    key =0
    print 'ingresa comando'
    while (key!='q'):
        key = raw_input()
        if key == 'd':
            motor1.dec()
        if key == 's':
            for i in range(30):
                motor1.dec()
                time.sleep(wait)
        if key == 'a':
            for i in range(300):
                motor1.dec()
                time.sleep(wait)
        if key == 'f':
            motor1.inc()

        if key == 'g':
            for i in range(30):
                motor1.inc()
                time.sleep(wait)
        if key == 'h':
            for i in range(300):
                motor1.inc()
                time.sleep(wait)
        if key == 'c':
            motor2.dec()
        if key == 'x':
            for i in range(30):
                motor2.dec()
                time.sleep(wait)
        if key == 'z':
            for i in range(300):
                motor2.dec()
                time.sleep(wait)
        if key == 'v':
            motor2.inc()

        if key == 'b':
            for i in range(30):
                motor2.inc()
                time.sleep(wait)
        if key == 'n':
            for i in range(300):
                motor2.inc()
                time.sleep(wait)
        print 'hecho'
    GPIO.cleanup()

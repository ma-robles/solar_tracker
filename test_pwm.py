import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BCM)
GPIO.setup(17,GPIO.OUT)
GPIO.setup(4, GPIO.OUT)

GPIO.output(4, GPIO.HIGH)
p = GPIO.PWM(17, 50)

p.start(50)
sleep(10)
p.stop()
GPIO.output(4, GPIO.LOW)
p.start(50)
sleep(10)
p.stop()
GPIO.cleanup()


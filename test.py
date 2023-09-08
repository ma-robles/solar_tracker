import picamera
import time

camera = picamera.PiCamera()
camera.start_preview()
print camera.resolution
time.sleep(3)
while(True):
    pass

from picamera import PiCamera
from time import sleep

# initialize camera
camera = PiCamera()
camera.resolution = (320, 240)
camera.led = False #save battery whenever possible
camera.rotation = 180

# short demo
camera.start_preview(alpha=200)
sleep(5)
camera.capture('image.jpg')
camera.stop_preview()

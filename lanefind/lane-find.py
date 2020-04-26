# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np #need for Hough transform

# resolution
WIDTH = 320
HEIGHT = 240

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (WIDTH, HEIGHT)
camera.framerate = 32
camera.rotation = 180
rawCapture = PiRGBArray(camera, size=(WIDTH, HEIGHT))

# allow the camera to warmup
time.sleep(0.1)

# apply Hough transform to processed image, and draw lines on base image
def find_lines(disp_img, proc_img):
  lines = cv2.HoughLinesP(proc_img, 1, np.pi/180, 200)

  for line in lines:
    x1, y1, x2, y2 = line[0]
    #draw lines on image
    cv2.line(disp_img, (x1, y1), (x2, y2), (0, 0, 255), 2)
  return disp_img

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
  # grab the raw NumPy array representing the image, then initialize the timestamp
  # and occupied/unoccupied text
  image = frame.array

  # mask the bottom two thirds of the image,
  # manipulate just this bottom with the relevant lane markings
  # BGR -> GRAY -> GaussianBlur -> Contrast -> Threshold

  #img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) #convert image to HSV colorspace
  #mask = cv2.inRange(img_hsv, (0, 0, )

  img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) #convert the image into black and white colorspace
  img_blur = cv2.GaussianBlur(img_gray, (9, 9), 0) #blur the image to cut out noise
  img_cont = cv2.equalizeHist(img_blur) #ehance contrast in image
  img_thresh = cv2.threshold(img_cont, 200, 255, cv2.THRESH_BINARY)[1]    
  image = find_lines(image, img_thresh)
  
  # show the frame
  cv2.imshow("Lanes", image)
  cv2.imshow("Processed", img_thresh)
  key = cv2.waitKey(1) & 0xFF #display the stream for 1ms and capture any key input as a character

  # clear the stream in preparation for the next frame
  rawCapture.truncate(0)

  # if the `q` key was pressed, break from the loop
  if key == ord("q"):
    break

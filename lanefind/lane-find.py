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

# apply probabilistic Hough transform to the processed ROI
def find_lines(roi):
  return cv2.HoughLinesP(roi, 1, np.pi/180, 200)

# draw lines on image with respect to a specific origin
def draw_lines(img, lines, x, y):
  for line in lines:
    x1, y1, x2, y2 = line[0]
    #draw lines on image
    cv2.line(img, (x1+x, y1+y), (x2+x, y2+y), (0, 0, 255), 2)
  return img
 

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
  img = frame.array

  # make a ROI for the bottom half of the image
  roi = img[HEIGHT/2:HEIGHT, 0:WIDTH] 

  # process the ROI for line finding
  roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY) #convert the image into black and white colorspace
  roi_blur = cv2.GaussianBlur(roi_gray, (9, 9), 0) #blur the image to cut out noise
  roi_cont = cv2.equalizeHist(roi_blur) #ehance contrast in image
  roi_thresh = cv2.threshold(roi_cont, 200, 255, cv2.THRESH_BINARY)[1]    
  
  # find lines with a Hough transform and draw them on the base image
  lines = find_lines(roi_thresh)
  img_line = draw_lines(img, lines, 0, HEIGHT/2)
  
  # show the frame
  cv2.imshow("Lanes", img_line)
  cv2.imshow("Processed", roi_thresh)
  key = cv2.waitKey(1) & 0xFF #display the stream for 1ms and capture any key input as a character

  # clear the stream in preparation for the next frame
  rawCapture.truncate(0)

  # if the `q` key was pressed, break from the loop
  if key == ord("q"):
    break

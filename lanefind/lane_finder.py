# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np #need for Hough transform

# debug param used for viewing
DEBUG = True

# resolution
WIDTH = 640
HEIGHT = 480

# initialize the camera and raw capture
camera = PiCamera()
camera.resolution = (WIDTH, HEIGHT)
camera.framerate = 32
camera.rotation = 180
rawCapture = PiRGBArray(camera, size=(WIDTH, HEIGHT))

# adjust the gamma of the image
def adjust_gamma(img, gamma=1.0):
  gamma_inv = 1.0/gamma
  table = np.array([
    ((i/255.0)**gamma_inv) * 255
    for i in np.arange(0, 256)])
  return cv2.LUT(img.astype(np.uint8), table.astype(np.uint8))

# isolate blue parts of the image
def isolate_blue(img):
  # define blue thresholds
  blue_min = (96, 130, 50)
  blue_max = (130, 255, 255)
  # mask off areas within the threshold
  img_gamma = adjust_gamma(img, 1.0)
  img_hsv = cv2.cvtColor(img_gamma, cv2.COLOR_BGR2HSV)
  img_blur = cv2.GaussianBlur(img_hsv, (9, 9), 0)
  mask = cv2.inRange(img_blur, blue_min, blue_max)
  img_proc = mask
  # return a binary image
  return img_proc

# minimize found shapes
def erode_image(img):
  kernel = np.ones((3,3), np.uint8)
  img_erode = cv2.erode(img, kernel, iterations=5)
  return img_erode

# apply a probabilistic Hough transform to the image
def find_lines(img):
  return cv2.HoughLinesP(img, 1, np.pi/180, 55, minLineLength=50, maxLineGap=5)

# draw found lines on image
def draw_lines(img, lines, x, y, color=(0, 0, 255)):
  if lines is not None:
    for line in lines:
      x1, y1, x2, y2 = line[0]
      cv2.line(img, (x1+x, y1+y), (x2+x, y2+y), color, 2)
  return img

# find the average angle based on the average slope of lines
def average_angle(lines):
  # by default go forward, not sure how else to approach this
  if lines is None: 
    #TODO log errors, should stop motor if this exceeds a threshold
    print 'no lines detected'    
    return 90
  slopes = []
  for line in lines:
    x1, y1, x2, y2 = line[0]
    if x1 != x2:
      slope = ((y2 - y1) / float(x2 - x1))
      slopes.append(slope)
      # TODO either log or disregard
      print 'x1:', x1, 'x2:', x2, 'y1:', y1, 'y2:', y2
  slope_avg = np.sum(slopes) / len(slopes)
  angle = np.arctan(slope_avg) * 180/np.pi
  # correct for change in coordinate system
  if (angle < 0):
    angle = angle * -1
  else:
    angle = 180 - angle
  print 'angle:', angle #TODO log this angle, for steering servo
  return angle

def birds_eye_view(img):
  #points in camera view, corresponding to empirical observation
  points1 = np.float32([[188, 171], [452, 171], [23, 374], [617, 374]])
  #points in bird's eye view
  points2 = np.float32([[171, 233], [469, 233], [171, 442], [469, 442]])
  #compute transformation from point sets
  T = cv2.getPerspectiveTransform(points1, points2)
  #execute transform on input image and store as new image
  img_trans = cv2.warpPerspective(img, T, (640, 480))
  return img_trans

def main():
  # allow the camera to warmup
  time.sleep(0.1)

  # capture frames from the camera
  for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image
    image = frame.array
  
    # transform image
    img_birds = birds_eye_view(image)

    # isolate blue areas in the bottom half of the transformed image
    img_half = img_birds[HEIGHT/2:HEIGHT, 0:WIDTH]
    img_blues = isolate_blue(img_half)
    img_erode = erode_image(img_blues)

    # find lines in ROI
    lines = find_lines(img_erode)

    # find average angle from lines
    angle = average_angle(lines)

    if DEBUG:
      # draw Hough lines on bird's eye view image
      img_lines = draw_lines(img_birds, lines, 0, HEIGHT/2)
      # show the frame
      cv2.imshow("Camera View", image)
      cv2.imshow("Bird's Eye View with Lane Lines", img_birds)
      cv2.waitKey(1) #required to display images, use Ctrl+C to quit
  
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)

if __name__ == '__main__':
  main()

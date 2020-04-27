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


# adjust gamma of an image
def adjust_gamma(img, gamma=1.0):
  gamma_inv = 1.0 / gamma
  table = np.array([
    ((i / 255.0) ** gamma_inv) * 255
    for i in np.arange(0, 256)])
  return cv2.LUT(img.astype(np.uint8), table.astype(np.uint8))


# process the image into HSV colorspace and isolate blue shapes
def process_image(img):
  img_gam = adjust_gamma(img, 1.0)
  img_hsv = cv2.cvtColor(img_gam, cv2.COLOR_BGR2HSV)
  img_blur = cv2.GaussianBlur(img_hsv, (9, 9), 0)
  blue_min = (88, 100, 50)
  blue_max = (138, 255, 255)
  mask = cv2.inRange(img_blur, blue_min, blue_max)
  img_proc = mask
  return img_proc


# apply a probabilistic Hough transform to an image
def find_lines(img):
  return cv2.HoughLinesP(img, 1, np.pi/180, 55, minLineLength=60, maxLineGap=5)


# draw found lines on image
def draw_lines(img, lines, x, y, color=(0, 0, 255)):
  if lines is not None:
    for line in lines:
      x1, y1, x2, y2 = line[0]
      cv2.line(img, (x1+x, y1+y), (x2+x, y2+y), color, 2)
  return img


# find the average slope, intercept and magnitude for each line
def avg_line_old(lines):
  #init lists
  acc = []
  weights = []
  #compute values
  for line in lines:
    x1, y1, x2, y2 = line[0]
    if x1 == x2:
      continue #ignore all vertical lines
    slope = (y2 - y1) / (x2 - x1)
    intercept = y1 - slope * x1
    magnitude = np.sqrt((y2 - y1)**2 + (x2 - x1)**2)
    acc.append((slope, intercept))
    weights.append((magnitude))
  avg = np.dot(weights, acc) / np.sum(weights) if len(weights) > 0 else None
  return avg


def avg_line(lines):
  x1s = y1s = x2s = y2s = 0
  for line in lines:
    x1, y1, x2, y2 = line[0]
    x1s += x1
    y1s += y1
    x2s += x2
    y2s += y2
  sums = [x1s, y1s, x2s, y2s]
  avgs =[]
  for i in sums:
    i /= len(lines)
    avgs.append(i)
  return avgs


# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
  img = frame.array

  # process image as to isolate lane markings
  img_proc = process_image(img)

  # define a left and right ROI for the car
  roi_l = img_proc[HEIGHT/2:HEIGHT, 0:WIDTH/2]
  roi_r = img_proc[HEIGHT/2:HEIGHT, WIDTH/2:WIDTH]

  # find lines in each ROI
  lines_l = find_lines(roi_l)
  lines_r = find_lines(roi_r)

  # find average line for each ROI
  if lines_l is not None:
    avg_l = avg_line(lines_l)
    #print('avg_l:', avg_l)
  if lines_r is not None:
    avg_r = avg_line(lines_r)
    #print('avg_r:', avg_r)

  # draw average lines on base image
  img_line = draw_lines(img, lines_l, 0, HEIGHT/2)
  img_line = draw_lines(img, lines_r, WIDTH/2, HEIGHT/2, (0, 255, 0))

  # show the frame
  cv2.imshow("Processed", img_proc)
  cv2.imshow("Lines", img_line)
  key = cv2.waitKey(1) & 0xFF #display the stream for 1ms and capture any key input as a character

  # clear the stream in preparation for the next frame
  rawCapture.truncate(0)

  # if the `q` key was pressed, break from the loop
  if key == ord("q"):
    break

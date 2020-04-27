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

# draw lanes on image with respect to a specific origin
def draw_lanes(img, lanes, x, y):
  #draw lines on blank image and then combine with base image
  img_line = np.zeros_like(img)
  for lane in lanes:
    if lane is not None:
      print('lane line: ', lane)
      (x1, y1), (x2, y2) = lane
      cv2.line(img_line, (x1+x, y1+y), (x2+x, y2+y), (0, 0, 255), 2)
  return cv2.addWeighted(img, 1.0, img_line, 0.95, 0.0)


# process a ROI in the image for line finding
def process_roi(img, x1, y1, x2, y2):  
  roi = img[y1:y2, x1:x2]
  roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY) #convert the image into black and white colorspace
  roi_blur = cv2.GaussianBlur(roi_gray, (9, 9), 0) #blur the image to cut out noise
  roi_thresh = cv2.threshold(roi_blur, 210, 255, cv2.THRESH_BINARY)[1]
  return roi_thresh


# adjust gamma of an image
def adjust_gamma(img, gamma=1.0):
  gamma_inv = 1.0 / gamma
  table = np.array([
    ((i / 255.0) ** gamma_inv) * 255
    for i in np.arange(0, 256)])
  return cv2.LUT(img.astype(np.uint8), table.astype(np.uint8))


# process the image into HSV colorspace and isolate blue shapes
def process_image(img):
  img_gam = adjust_gamma(img, 2.5)
  img_hsv = cv2.cvtColor(img_gam, cv2.COLOR_BGR2HSV)
  img_blur = cv2.GaussianBlur(img_hsv, (11, 11), 0)
  blue_min = (85, 0, 0)
  blue_max = (130, 255, 255)
  mask = cv2.inRange(img_blur, blue_min, blue_max)
  img_proc = mask
  return img_proc


# apply a probabilistic Hough transform to an image
def find_lines(img):
  return cv2.HoughLinesP(img, 1, np.pi/180, 65, minLineLength=0, maxLineGap=5)


# draw found lines on image
def draw_lines(img, lines):
  if lines is not None:
    for line in lines:
      x1, y1, x2, y2 = line[0]
      cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 1)
  return img



# find the average slope, intercept and magnitude for each line
def avg_slope_intercept(lines):
  #init lists
  l_lines = [] #(slope, intercept)
  l_weight = [] #(length,)
  r_lines = []
  r_weight = []
  #compute values
  for line in lines:
    for x1, y1, x2, y2 in line:
      if x1 == x2:
        continue #ignore all vertical lines
      slope = (y2 - y1) / (x2 - x1)
      intercept = y1 - slope * x1
      length = np.sqrt((y2 - y1)**2 + (x2 - x1)**2)
      if slope < 0:
        l_lines.append((slope, intercept))
	l_weight.append((length))
      else:
	r_lines.append((slope, intercept))
	r_weight.append((length))
  #weight lines with greater magnitude
  l = np.dot(l_weight, l_lines) / np.sum(l_weight) if len(l_weight) > 0 else None
  r = np.dot(r_weight, r_lines) / np.sum(r_weight) if len(r_weight) > 0 else None
  return l, r


# convert lines composed of slope and intercept into tuple of start and end points
def line_points(y1, y2, line):
  if line is None:
    return None
  #else
  slope, intercept = line
  x1 = int((y1 - intercept) / slope)
  x2 = int((y2 - intercept) / slope)
  y1 = int(y1)
  y2 = int(y2)
  return ((x1, y1), (x2, y2))


# get average lane lines from lines found with Hough transform
def lane_lines(lines, y1, y2):
  l_lane, r_lane = avg_slope_intercept(lines)
  l_line = line_points(y1, y2, l_lane)
  r_line = line_points(y1, y2, r_lane)
  return l_line, r_line


# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
  img = frame.array

  # process image as to isolate lane markings
  img_proc = process_image(img)

  # find lines in image
  lines = find_lines(img_proc)
  img_line = draw_lines(img, lines)

  # show the frame
  cv2.imshow("Processed", img_proc)
  cv2.imshow("Lines", img_line)
  key = cv2.waitKey(1) & 0xFF #display the stream for 1ms and capture any key input as a character

  # clear the stream in preparation for the next frame
  rawCapture.truncate(0)

  # if the `q` key was pressed, break from the loop
  if key == ord("q"):
    break

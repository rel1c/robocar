# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np #need for Hough transform

# resolution
WIDTH = 640
HEIGHT = 480

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (WIDTH, HEIGHT)
camera.framerate = 32
camera.rotation = 180
rawCapture = PiRGBArray(camera, size=(WIDTH, HEIGHT))

# define a polygon for image transform
# polygon is based on a 17" x 12" rectangle
TOP_L = (188, 171)
TOP_R = (452, 171)
BOT_L = (23, 374)
BOT_R = (617, 374)

# draw a crosshair to aid in centering image
def draw_crosshair(img):
  color = (0, 0, 255)
  img_horizontal = cv2.line(img, (0, HEIGHT/2), (WIDTH, HEIGHT/2), color, 1)
  img_vertical = cv2.line(img_horizontal, (WIDTH,2, 0), (WIDTH/2, HEIGHT), color, 1)
  return img_vertical

# draw a polygon based on empirical measurements
def draw_base_polygon(img):
  color = (100, 255, 255)
  img_up = cv2.line(img, TOP_L, TOP_R, color, 1)
  img_down = cv2.line(img_up, BOT_L, BOT_R, color, 1)
  img_left = cv2.line(img_down, TOP_L, BOT_L, color, 1)
  img_right = cv2.line(img_left, TOP_R, BOT_R, color, 1)
  return img_right

# adjust the gamma of an image
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

# find lines in image using probabilistic Hough transform
def find_lines(img):
  return cv2.HoughLinesP(img, 1, np.pi/180, 55, minLineLength = 50, maxLineGap = 10)

# draw found lines on image
def draw_lines(img, lines, x, y, color=(0, 0, 255)):
  if lines is not None:
    for line in lines:
      x1, y1, x2, y2 = line[0]
      cv2.line(img, (x1+x, y1+y), (x2+x, y2+y), color, 2)
  return img

# find the average slope, intercept for a set of lines
def avg_slop_intercept(lines):
  #init lists
  acc = []
  mag = []
  #compute values
  for line in lines:
    x1, y1, x2, y2 = line[0]
    if x1 == x2:
      continue #ignore all horizontal lines
    a = ((y2 - y1) / float(x2 - x1))
    b = (y1 - a * x1)
    magnitude = np.sqrt((y2 - y1)**2 + (x2 - x1)**2)
    if a != 0:
      acc.append((a, b))
      mag.append((magnitude))
  avg = np.dot(mag, acc) / np.sum(mag) if len(mag) > 0 else None
  return avg

# convert a slope and intercept into a tuple of start and end points
def slope_to_points(line, y1=0, y2=HEIGHT):
  if line is None:
    return 0, 0, 1, 1
  #else
  a, b = line
  x1 = int((y1 - b) / a)
  x2 = int((y2 - b) / a)
  y1 = int(y1)
  y2 = int(y2)
  return x1, y1, x2, y2

# find the most confident steering angle from a the average slope and intercept
# of a set of lines. if there are no lines, the default is to point forward at
# 90 degress
def get_steer_angle(line):
  if line is None:
    return 90
  else:
    inter, theta = line
    return theta

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

    # find lines in ROI
    lines = find_lines(img_blues)

    # draw average lines on bird's eye view image
    img_lines = draw_lines(img_birds, lines, 0, HEIGHT/2, (255, 0, 0))

    # show the frame
    cv2.imshow("Image", image)
    cv2.imshow("Lines", img_lines)

    #cv2.imshow("Processed", img_thresh)
    key = cv2.waitKey(1) & 0xFF #display the stream for 1ms and capture any key input as a character
  
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
  
    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
      break

if __name__ == '__main__':
  main()

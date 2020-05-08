# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np #need for Hough transform

# debug param MOVE THIS
DEBUG = True

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

# minimize found lanes to small lines
def erode_image(img):
  kernel = np.ones((3,3), np.uint8)
  img_erode = cv2.erode(img, kernel, iterations=5)
  return img_erode

# find lines in image using probabilistic Hough transform
def find_lines(img):
  return cv2.HoughLines(img, 1, np.pi/180, 45)

# draw found line on image
def draw_line(img, line):
  color = (255, 200, 100)
  rho, theta = line
  a = np.cos(theta)
  b = np.sin(theta)
  x0 = a*rho
  y0 = b*rho
  x1 = int(x0 + 1000*(-b))
  y1 = int(y0 + 1000*(a))
  x2 = int(x0 - 1000*(-b))
  y2 = int(y0 - 1000*(a))
  cv2.line(img, (x1, y1), (x2, y2), color, 2)
  return img

def draw_lines(img, lines):
  if lines is not None:
    print len(lines)
    for line in lines[0]:
      img = draw_line(img, line)
  return img

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

    if DEBUG:
      # draw Hough lines on bird's eye view image
      img_lines = draw_lines(img_erode, lines)
      # show the frame
      #cv2.imshow("Image", image)
      cv2.imshow("Isolated", img_erode)
      cv2.waitKey(1)
  
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)

if __name__ == '__main__':
  main()

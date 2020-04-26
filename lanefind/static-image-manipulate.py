import cv2
import numpy as np

def nothing(x):
   pass

# create a window
cv2.namedWindow('image')

# create trackbars for color change
cv2.createTrackbar('Gaussian Blur','image',1,35,nothing) #9
cv2.createTrackbar('Threshold','image',0,255,nothing) #210
cv2.createTrackbar('Magnitude','image',0,400,nothing)
cv2.setTrackbarPos('Gaussian Blur','image',9)
cv2.setTrackbarPos('Threshold','image',210)
cv2.setTrackbarPos('Magnitude','image',200)

# apply Hough transform to processed image, and draw lines on base image
def find_lines(disp_img, proc_img):
  l = cv2.getTrackbarPos('Magnitude','image')  
  lines = cv2.HoughLines(proc_img, 1, np.pi/180, l)

  if(lines is not None):
    for line in lines:
      rho,theta = line[0]
      a = np.cos(theta)
      b = np.sin(theta)
      x0 = a*rho
      y0 = b*rho
      x1 = int(x0 + 1000*(-b))
      y1 = int(y0 + 1000*(a))
      x2 = int(x0 - 1000*(-b))
      y2 = int(y0 - 1000*(a))
      #draw lines on image
      cv2.line(disp_img, (x1, y1), (x2, y2), (0, 0, 255), 2)
  return disp_img

while(1):
   img = cv2.imread('image.jpg')
   k = cv2.waitKey(1) & 0xFF
   if k == ord('q'):
       break

   # get current positions of four trackbars
   g = cv2.getTrackbarPos('Gaussian Blur','image')
   t = cv2.getTrackbarPos('Threshold','image')

   # gaussian blur needs odd values
   if (g % 2 == 0):
       g = g + 1
       cv2.setTrackbarPos('Gaussian Blur','image',g)

   # adjust image
   img_part = img[80:240, 0:340] #x:x+w, y:y+h
   img_gray = cv2.cvtColor(img_part, cv2.COLOR_BGR2GRAY)
   img_blur = cv2.GaussianBlur(img_gray, (g, g), 0)
   img_eqlz = cv2.equalizeHist(img_blur)
   img_thrs = cv2.threshold(img_eqlz, t, 255, cv2.THRESH_BINARY)[1]

   # find lines in image
   img_proc = find_lines(img_part, img_thrs)
   img_show = img
   img_show[80:240, 0:340] = img_proc

   cv2.imshow('image', img_show)

cv2.destroyAllWindows()

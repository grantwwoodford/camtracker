import numpy as np
import cv2

# copy parameters to arrays
K = np.array([[791.05512935, 0, 337.7480753], [0, 786.95438005, 207.14814042], [0, 0, 1]])
d = np.array([0.11597251, -0.96652378, 0, 0, 0]) # just use first two terms (no translation)

# read one of your images
img = cv2.imread("/home/grant/Documents/calibrations/pic00039.jpg")
h, w = img.shape[:2]

# undistort
newcamera, roi = cv2.getOptimalNewCameraMatrix(K, d, (w,h), 0) 
newimg = cv2.undistort(img, K, d, None, newcamera)

cv2.imwrite("original.jpg", img)
cv2.imwrite("undistorted.jpg", newimg)

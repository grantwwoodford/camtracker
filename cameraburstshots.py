import cv2.cv as cv
import time

cv.NamedWindow("camera", 1)

capture = cv.CaptureFromCAM(1)

cv.SetCaptureProperty(capture,cv.CV_CAP_PROP_FRAME_WIDTH, 800)
cv.SetCaptureProperty(capture,cv.CV_CAP_PROP_FRAME_HEIGHT, 600)

i = 0
j = 0
while True:
    img = cv.QueryFrame(capture)
    cv.ShowImage("camera", img)

    if i % 20 == 0:
        cv.SaveImage('/home/grant/Documents/calibrations/pic{:>05}.jpg'.format(j), img)
        j += 1
    if cv.WaitKey(33) == 27:
        break
    i += 1
    

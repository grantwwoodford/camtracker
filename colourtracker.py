import sys
import getopt
import cv2
import numpy as np
import copy
import math
import os.path
from math import atan2, degrees, pi
from stopwatch import get_elapsedtime
from bisect import bisect_left
import threading
from time import sleep

lock = threading.Lock()

positions = []
timestamp_keys = []

calibration_ration = 5
calibrated_position = (0,0,0,0)
calibrated = False

transform = None
transform_size = None

# variable containing the 4 points selected by the user in the corners of the board
corner_point_list = []

camera_id = 1
tracking_mode = 0

##
# Creates a new RGB image of the specified size, initially
# filled with black.
##
def new_rgb_image(width, height):
    image = np.zeros( (height, width, 3), np.uint8)
    return image
 
##
# This function is called by OpenCV when the user clicks
# anywhere in a window displaying an image.
##
def mouse_click_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print "Click at (%d,%d)" % (x,y)
        corner_point_list.append( (x,y) )

##
# Computes a perspective transform matrix by capturing a single
# frame from a video source and displaying it to the user for
# corner selection.
#
# Parameters:
# * dev: Video Device (from open_camera())
# * board_size: A tuple/list with 2 elements containing the width and height (respectively) of the gameboard (in arbitrary units, like inches)
# * dpi: Scaling factor for elements of board_size
# * calib_file: Optional. If specified, the perspective transform matrix is saved under this filename.
#   This file can be loaded later to bypass the calibration step (assuming nothing has moved).
##
def get_transform_matrix(dev, board_size, dpi, calib_file = None):
    # if calibrations file exist, skip asking user for calibration points
    if (os.path.isfile(calib_file)): 
        trans = np.loadtxt(calib_file)
        return trans
    
    # Read a frame from the video device
    img = get_undistorted_frame(dev)
 
    # Displace image to user
    cv2.imshow("Calibrate", img)
 
    # Register the mouse callback on this window. When 
    # the user clicks anywhere in the "Calibrate" window,
    # the function mouse_click_callback() is called (defined above)
    cv2.setMouseCallback("Calibrate", mouse_click_callback)
 
    # Wait until the user has selected 4 points
    while True:
        # If the user has selected all 4 points, exit loop.
        if (len(corner_point_list) >= 4):
            print "Got 4 points: "+str(corner_point_list)
            break
 
        # If the user hits a key, exit loop, otherwise remain.
        if (cv2.waitKey(10) >= 0):
            break;
 
    # Close the calibration window:
    cv2.destroyWindow("Calibrate")
 
    # If the user selected 4 points
    if (len(corner_point_list) >= 4):
        # Do calibration
 
        # src is a list of 4 points on the original image selected by the user
        # in the order [TOP_LEFT, BOTTOM_LEFT, TOP_RIGHT, BOTTOM_RIGHT]
        src = np.array(corner_point_list, np.float32)
 
        # dest is a list of where these 4 points should be located on the
        # rectangular board (in the same order):
        dest = np.array( [ (0, 0), (0, board_size[1]*dpi), (board_size[0]*dpi, 0), (board_size[0]*dpi, board_size[1]*dpi) ], np.float32)
 
        # Calculate the perspective transform matrix
        trans = cv2.getPerspectiveTransform(src, dest)
 
        # If we were given a calibration filename, save this matrix to a file
        if calib_file:
            np.savetxt(calib_file, trans)
        return trans
    else:
        return None

# set video frame size
def open_camera(cam_id = 0):
    # create video capture
    cap = cv2.VideoCapture(cam_id)
    cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 600)
    cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 800)
    cap.set(cv2.cv.CV_CAP_PROP_FPS, 60)

    return cap

def get_undistorted_frame(device):
    _,frame = device.read()

    #cv2.imshow('frame2',frame)

    # copy calibration parameters to arrays
    K = np.array([[875.70630229, 0, 372.04452368], [0, 881.10303569, 310.62758208], [0, 0, 1]])
    d = np.array([3.05367078e-01, -1.10370447, 0, 0, 0])

    h, w = frame.shape[:2]
    
    # undistort
    newcamera, roi = cv2.getOptimalNewCameraMatrix(K, d, (w,h), 0)
    newimg = cv2.undistort(frame, K, d, None, newcamera)
    frame = newimg
    return frame

def get_frame(device):
    frame = get_undistorted_frame(device)

    return frame

def add_position(position):
    global positions
    global timestamp_keys

    ts= int(position[0])
    x = int(position[1])
    y = int(position[2])
    a = int(position[3])
    x2= int(position[4])
    y2= int(position[5])

    # elapsed_time, x1, y1, angle, x2, y2
    tuptup = (ts, x, y, a, x2, y2)

    #print len(positions)

    # ensure thread safe
    positions.append(tuptup) 
    timestamp_keys.append(ts)
        
def take_interpolated_value(myNumber):
    # only search timestamp_keys list while not currently being edited
    pos = bisect_left(timestamp_keys, myNumber)

    # timestamp has not been captured yet
    timeout_count = 0
    while pos == len(timestamp_keys) or pos == len(timestamp_keys):
        # create timeout so that does not hold up server
        if (timeout_count > 80):
            print 'timeout reached'
            return not_found_position(myNumber)
        pos = bisect_left(timestamp_keys, myNumber)            
        sleep(0.005)
        timeout_count += 1

    if len(timestamp_keys) == 0:
        print 'position not found'
        return not_found_position(myNumber)

    before = timestamp_keys[pos - 1]
    after = timestamp_keys[pos]
    
    if before == after:
        return format_position(myNumber, positions[pos-1])

    # apply interpolation
    total_time_gap = (myNumber - before)
    time_percentage = total_time_gap/(after - before)

    # sampling rate is too low
    if total_time_gap > 500:
        print 'total timegap reached: ' + str(myNumber)
        return not_found_position(myNumber)
    
    #print 'positions length: ' + str(len(positions)) + ', ' + str(len(timestamp_keys)) + ',' + str(pos)
    x = positions[pos-1][1] + (positions[pos][1] - positions[pos-1][1])*time_percentage
    y = positions[pos-1][2] + (positions[pos][2] - positions[pos-1][2])*time_percentage
    
    angle1 = positions[pos][3]
    angle2 = positions[pos-1][3]
    angle_difference = angle2 - angle1

    x2 = 0
    y2 = 0
    if tracking_mode == 1:
        x2 = positions[pos-1][4] + (positions[pos][4] - positions[pos-1][4])*time_percentage
        y2 = positions[pos-1][5] + (positions[pos][5] - positions[pos-1][5])*time_percentage

    if angle_difference > 180:
        angle1 = angle1 + 360
    elif angle_difference < -180:
        angle2 = angle2 + 360
    
    a = angle2 + (angle1 - angle2)*time_percentage
    
    returnValue = format_position(myNumber, (myNumber, x, y, a % 360, x2, y2))

    return returnValue 

def format_position(time_stamp, position):
    if tracking_mode == 0:
        return (time_stamp, position[1], position[2], position[3])
    else:
        return (time_stamp, position[1], position[2], position[3], position[4], position[5])

def not_found_position(time_stamp):
    if tracking_mode == 0:
        return (time_stamp, -9999, -9999, -9999)
    else:
        return (time_stamp, -9999, -9999, -9999, -9999, -9999)

def get_last_position():
    return positions[len(positions)-1]

def find_centroid(threshold, frame):
    # find contours in the threshold image
    contours,hierarchy = cv2.findContours(threshold,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    best_cnt = find_best_center(contours)

    if best_cnt is not None:
        M = cv2.moments(best_cnt)
        cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
    else:
        cx,cy = (None,None)
        
    return (cx, cy)

# TODO: only need rectangle for debugging purposes
def find_rectangle(threshold, frame):
    # find contours in the threshold image
    contours,hierarchy = cv2.findContours(threshold,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    best_cnt = find_best_center(contours)

    if best_cnt is not None:
        x,y,w,h = cv2.boundingRect(best_cnt)
    else:
        x,y,w,h = (0,0,0,0)
        
    return (x,y,w,h)

def find_best_center(contours):
    # finding contour with maximum area and store it as best_cnt
    best_cnt = None
    
    max_area = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > max_area:
            max_area = area
            best_cnt = cnt

    return best_cnt

def draw_directed_line(frame, x1, y1, x2, y2):
    if x1 is None or y1 is None or x2 is None or y2 is None:
        return

    cv2.line(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0,255,0), 2, 4)
    #cv2.circle(frame,(int(x1), int(y1)),5,255,-1)

def draw_path(frame):
    cur_pos = positions[-min(100, len(positions)-1)]

    for x in positions[-min(100, len(positions)-1):]:
        draw_directed_line(frame, cur_pos[1], cur_pos[2], x[1], x[2])
        cur_pos = x

def reset_positions():
    global positions
    global timestamp_keys
    positions = []
    timestamp_keys = []

def set_calibration(x_origin, y_origin):
    # calibrate
    global calibrated_position
    global calibration_ration
    global positions
    global timestamp_keys
    reset_positions()
    calibrated_position = (x_origin, y_origin)
    print calibrated_position
    
    # determined manually from undistorted image and calibration points
    calibration_ration = 2.75
    print 'calibration ratio: ' + str(calibration_ration)

def apply_calibration(uncalibrated_position):
    # y-direction has a negative sign on purpose to flip it
    calibrated = (uncalibrated_position[0],
                  (uncalibrated_position[1] - calibrated_position[0])/calibration_ration,
                  -(uncalibrated_position[2] - calibrated_position[1])/calibration_ration, uncalibrated_position[3],
                  (uncalibrated_position[4] - calibrated_position[0])/calibration_ration,
                  -(uncalibrated_position[5] - calibrated_position[1])/calibration_ration)

    # print "apply_calibration: " +  str(calibrated)
    return calibrated

def formatted_sending_position(timestamp, position):
    if tracking_mode == 0:
        return "TS:" + str(position[1]) +":"+ str(position[2]) +":"+ str(position[3]) + ":" + str(timestamp)
    else:
        return "TS:" + str(position[1]) +":"+ str(position[2]) +":"+ str(position[3]) +":"+ str(position[4]) +":"+ str(position[5]) + ":" + str(timestamp)

def reset_calibration():
    global calibrated
    calibrated = 0

def calculate_angle(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    rads = atan2(dy,dx)
    rads %= 2*pi
    degs = degrees(rads) + 90

    if degs < 0:
        degs = degs + 360
    elif degs > 360:
        degs = degs - 360

    return degs
 
def get_thresholds(hsv, sat_point1, sat_point2, sat_range):
    # origin colour
    thresh_direction = cv2.inRange(hsv,np.array((sat_point1-sat_range, 50, 50)), np.array((sat_point1+sat_range, 255, 255)))

    # direction colour
    thresh_origin = cv2.inRange(hsv,np.array((sat_point2-sat_range, 100, 100)), np.array((sat_point2+sat_range, 255, 255)))
    #cv2.imshow('frame_origin',thresh_origin)

    return (thresh_origin, thresh_direction)

def trackingthread(argv):
    global positions 
    global calibrated
    global fs
    global transform
    global transform_size

    process_arguments(argv)

    # The size of the board in inches, measured between the two
    # robot boundaries:
    board_size = [16, 16]
 
    # Number of pixels to display per inch in the final transformed image. This
    # was selected somewhat arbitrarily (I chose 17 because it fit on my screen):
    dpi = 17

    transform_size = (int(board_size[0]*dpi), int(board_size[1]*dpi))
    cap = open_camera(camera_id)

    # Calculate the perspective transform matrix
    transform = get_transform_matrix(cap, board_size, dpi, 'calibrations.txt')

    # saturation ranges for targeting desired colour
    sat1 = 70
    sat2 = 40
    sat_range = 20
    
    cv2.namedWindow('frame', cv2.CV_WINDOW_AUTOSIZE)

    snake_rectangle_list = []
    snake_centroid_list = []

    show_rectangle = True
    show_centroid = True

    # can take a while for camera to focus on first use
    # code below allows for a 2 second delay
    frame = get_frame(cap)
    sleep(1)
    frame = get_frame(cap)
    sleep(1)

    while(1):

        # read the frame
        frame = get_frame(cap)
        frame_no_blur = frame.copy()

        # smooth it
        frame = cv2.blur(frame,(3,3))

        # shows frame without drawings or edits
        #cv2.imshow('frame2',frame)

        # two-point colour tracking
        # convert to hsv and find range of colors
        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        thresh_origin, thresh_direction = get_thresholds(hsv,sat1,sat2,sat_range)

        #cv2.imshow('dire',thresh_direction)
        cx_origin, cy_origin = find_centroid(thresh_origin, frame)
        cx_direction, cy_direction = find_centroid(thresh_direction, frame)

        if show_centroid:
            if cx_origin is not None and cx_direction is not None:
                # finding centroids of best_cnt and draw a circle there
                cv2.circle(frame,(cx_origin,cy_origin),5,255,-1)
                cv2.circle(frame,(cx_direction,cy_direction),5,255,-1)

                # draw line
                draw_directed_line(frame_no_blur, cx_origin, cy_origin, cx_direction, cy_direction)

        # calibrate
        if not calibrated and (cx_origin, cy_origin) != (0,0):
            set_calibration(cx_origin, cy_origin)
            calibrated = True

        if cx_origin is not None and cx_direction is not None:
            # work out current angle
            angle = calculate_angle(cx_origin, cy_origin, cx_direction, cy_direction)

            # add position to list
            add_position((get_elapsedtime(), cx_origin, cy_origin, angle, cx_direction, cy_direction))

            # draw path, the last 100 positions
            #draw_path(frame)

            # print position details
            draw_text(frame_no_blur, (get_elapsedtime(), cx_origin, cy_origin, angle, cx_direction, cy_direction));

        # display current frame
        cv2.imshow('frame', frame_no_blur)

        # Show it, if key pressed is 'Esc', exit the loop
        k = cv2.waitKey(1)
        #print k
        if k == 27: # esc
            break
        elif k == 114: # r
            reset_calibration()
        elif k  == 97: # a
            sat1 += 5
        elif k  == 122: # z
            sat1 -= 5
        elif k  == 115: # s
            sat2 += 5
        elif k  == 120: # x
            sat2 -= 5

        # print str(sat1) + ', ' + str(sat2)
    
    # Clean up everything before leaving
    cleanup(cap)
        
    # Log positions to file
    write_positions_to_file()


# Closes all OpenCV windows and releases video capture device before exit.
def cleanup(cap): 
    cv2.destroyAllWindows()
    cap.release()

def write_positions_to_file():
    f = open('positions_log.txt','w')
    f.write('ts,x,y,a,x2,y2\n')

    for cur_pos in positions:
        cur_pos = apply_calibration(cur_pos)
        f.write(str(cur_pos[0]) + ',' + str(cur_pos[1]) + ',' + str(cur_pos[2]) + ',' + str(cur_pos[3]) +
                 ',' + str(cur_pos[4]) + ',' + str(cur_pos[5]) + '\n')

    f.close()

def draw_text(frame, position):
    text = map(toInt, apply_calibration(position));
    cv2.putText(frame, str(text), (50,100), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0,0,255))

def toInt(x): return int(x)

def process_arguments(argv):
    global camera_id
    global tracking_mode

    # set camera to its index
    camera_id = 1
    tracking_mode = 0

    try:
        opts, args = getopt.getopt(argv,"hi:o:",["icamera_id=","otracking_mode="])
    except getopt.GetoptError:
        print 'server.py -i <camera_id> -o <tracking_mode>'
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print 'server.py -i <camera_id> -o <tracking_mode>'
            sys.exit()
        elif opt in ("-i", "--icamera_id"):
            camera_id = int(arg)
        elif opt in ("-o", "--otracking_mode"):
            tracking_mode = int(arg)
    print 'Camera mode is ', camera_id
    print 'Output file is ', tracking_mode
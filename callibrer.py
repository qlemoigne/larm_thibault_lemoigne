#!/usr/bin/env python3
## Doc: https://dev.intelrealsense.com/docs/python2

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import time, numpy as np
import sys, cv2
import matplotlib.pyplot as pyplot
# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

print( f"Connect: {device_product_line}" )
found_rgb = True
for s in device.sensors:
    print( "Name:" + s.get_info(rs.camera_info.name) )
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True

if not (found_rgb):
    print("Depth camera equired !!!")
    exit(0)

config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 60)
config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 60)




# Start streaming
pipeline.start(config)

class Filter():
    def __init__(self, low, high):
        self.low = low
        self.high = high

orangeFilters = []
blackFilters = []

def souris(event, x, y, flags, param):
    global image_hsv



    if event==cv2.EVENT_LBUTTONDOWN:
        h = image_hsv[y, x][0]
        s = image_hsv[y, x][1]
        v = image_hsv[y, x][2]
        
        print("COlor: " + str(h) + ", " + str(s) + ", " + str(v))

        filter = Filter(np.array([h - 2, s - 2, v - 4]), np.array([h + 2, s +   2, v + 4]))

        orangeFilters.append(filter)

try:
    count= 1
    refTime= time.process_time()
    freq= 30

    sys.stdout.write("-")

    kernel = np.ones((3, 3), np.uint8)

    

    while True:

        # Wait for a coherent tuple of frames: depth, color and accel
        frames = pipeline.wait_for_frames()





        depth_frame = frames.first(rs.stream.depth)
        color_frame = frames.first(rs.stream.color)

        if not (depth_frame and color_frame):
            continue
        
        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        
        image_hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV) # Convert bgr color scheme to hsv

        
        mask = cv2.inRange(image_hsv, np.array([0,0,0]), np.array([0,0,0]))

        for filter in orangeFilters:
            temp_mask = cv2.inRange(image_hsv, filter.low, filter.high)
            mask = cv2.add(mask, temp_mask)

            

        mask = cv2.erode(mask, kernel=kernel, iterations=1)

        
        # Show images
        cv2.namedWindow('Calibration', cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback('Calibration', souris)
        cv2.imshow('Calibration', color_image)
        cv2.imshow('Masque', mask)
        #cv2.imshow('Résultat', mask)
        cv2.waitKey(1)
        
        # Frequency:
        if count == 10 :
            newTime= time.process_time()
            freq= 10/((newTime-refTime))
            refTime= newTime
            count= 0
        count+= 1

finally:
    # Stop streaming
    print("\nEnding...")
    pipeline.stop()

#!/usr/bin/env python3
## Doc: https://dev.intelrealsense.com/docs/python2

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import time, numpy as np
import sys, cv2
import matplotlib.pyplot as pyplot

import math
import matplotlib.pyplot as plt
from skimage.measure import label, regionprops
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

'''class Filter():
    def __init__(self, low, high):
        self.low = low
        self.high = high'''

orangeFilters = []
blackFilters = []

def souris(event, x, y, flags, param):
    global image_hsv



    if event==cv2.EVENT_LBUTTONDOWN:
        h = image_hsv[y, x][0]
        s = image_hsv[y, x][1]
        v = image_hsv[y, x][2]
        
        print("COlor: " + str(h) + ", " + str(s) + ", " + str(v))

        filter = np.array([h, s, v]);
        #, np.array([h + 2, s +   2, v + 4]))

        orangeFilters.append(filter)

    if event==cv2.EVENT_RBUTTONDOWN:
        print("Résultat calibration : code à ajouter dans objects.py dans la section CRITERES DE CALIBRATION")
        print("")

        orangeArray = "self.orangeFilters = ["

        for filter in orangeFilters:
            orangeArray += "np.array([" + str(filter[0]) +"," + str(filter[1]) +"," + str(filter[2]) +"]),"

        orangeArray += "]"

        print(orangeArray)

        low = np.array([cv2.getTrackbarPos('H-', 'Calibration'),cv2.getTrackbarPos('S-', 'Calibration'),cv2.getTrackbarPos('delta V', 'Calibration')])
        high = np.array([cv2.getTrackbarPos('H+', 'Calibration'),cv2.getTrackbarPos('S+', 'Calibration'),cv2.getTrackbarPos('delta V', 'Calibration')])

        print("self.orangeLow = np.array([" + str(low[0]) +"," + str(low[1]) + "," + str(low[2]) + "])")
        print("self.orangeHigh = np.array([" + str(high[0]) +"," + str(high[1]) + "," + str(high[2]) + "])")

        exit(0)


def onTrackChange(x):
    pass

try:
    count= 1
    refTime= time.process_time()
    freq= 30

    sys.stdout.write("-")

    kernel2 = np.ones((2, 2), np.uint8)
    kernel3 = np.ones((3, 3), np.uint8)
    kernel4 = np.ones((4, 4), np.uint8)
    kernel6 = np.ones((6, 6), np.uint8)

    
    cv2.namedWindow('Calibration', cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback('Calibration', souris)
    cv2.createTrackbar('H-', 'Calibration', 4, 8,onTrackChange)
    cv2.createTrackbar('H+', 'Calibration', 4, 8,onTrackChange)
    cv2.createTrackbar('S-', 'Calibration', 35, 40,onTrackChange)
    cv2.createTrackbar('S+', 'Calibration', 35, 40,onTrackChange)
    cv2.createTrackbar('delta V', 'Calibration',60, 70,onTrackChange)
    #cv2.createTrackbar('V+', 'Calibration', 0, 64,onTrackChange)

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

        low = np.array([cv2.getTrackbarPos('H-', 'Calibration'),cv2.getTrackbarPos('S-', 'Calibration'),cv2.getTrackbarPos('delta V', 'Calibration')])
        high = np.array([cv2.getTrackbarPos('H+', 'Calibration'),cv2.getTrackbarPos('S+', 'Calibration'),cv2.getTrackbarPos('delta V', 'Calibration')])

        for filter in orangeFilters:
            temp_mask = cv2.inRange(image_hsv, filter - low, filter + high)
            mask = cv2.add(mask, temp_mask)

            

        mask = cv2.erode(mask, kernel=kernel2, iterations=4)
        mask = cv2.dilate(mask, kernel=kernel4, iterations=5)

        mask = cv2.erode(mask, kernel=kernel6, iterations=2)
        mask = cv2.dilate(mask, kernel=kernel4, iterations=2)
        
        # Show images
        '''elements = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]



        if len(elements) > 0:
            c = max(elements, key=cv2.contourArea)
            x,y,w,h = cv2.boundingRect(c)
            #areas = []

            #print("cc")

            if w > 10 and y < 500:
                cv2.rectangle(color_image, (x, y), (x + w,y + h), (0, 0, 255), 2)
                print("Found orange bottle : " + str(x) + " , " + str(y))'''

        label_image = label(mask)
        regions = regionprops(label_image)
        #print(len(regions))


        for props in regions:
            y0, x0 = props.centroid


            minr, minc, maxr, maxc = props.bbox

            rsize = maxr - minr
            csize = maxc - minc

            ratio = csize / rsize

            if csize > rsize:
                continue

            if props.extent > 0.85 or props.extent < 0.35:
                continue


            # test horizontal

            if ratio <= 0.5 and ratio >= 0.3:
                
                perimetre = 2 * rsize + 2 * csize

                if perimetre > 1000 or perimetre < 90:
                    continue


                cv2.rectangle(color_image, (int(minc), int(minr)), (int(maxc) ,int(maxr)), (255, 0, 0), 2)
                
                print("new !!")

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

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


#lo = np.array([2, 150, 50])
#hi = np.array([13, 255, 255])

#lo = np.array([0, 0, 0])
#hi = np.array([255, 255, 255])

DST = 10

class Area():

    def __init__(self, minx, maxx, miny, maxy):



        self.minx = minx
        self.maxx = maxx
        self.miny = miny
        self.maxy = maxy


    def isCommon(self, area):
        
        if ( (area.minx >= self.minx - DST and area.minx <= self.maxx + DST) or (area.maxx >= self.minx - DST and area.maxx <= self.maxx + DST)) and ((area.miny >= self.miny - DST and area.miny <= self.maxy + DST) or (area.maxy >= self.miny - DST and area.maxy <= self.maxy + DST)):
            return True


        return False    
    
    def isEmpty(self):
        return self.sizeX() * self.sizeY() < 100

    def sizeX(self):
        return self.maxx - self.minx

    def sizeY(self):
        return self.maxy - self.miny

    def merge(self, area):

        newArea = Area(minx= min(self.minx, area.minx), maxx= max(self.maxx, area.maxx), miny= min(self.miny, area.miny), maxy= max(self.maxy, area.maxy))
        

        return newArea

    ''' renvoit si cette area est plus petite que celle en param'''
    def isSmaller(self, area):
        return self.sizeX() * self.sizeY() < area.sizeX() * area.sizeY()

hsv_px = [0,0,0]

def souris(event, x, y, flags, param):
    global lo, hi, color, hsv_px

    if event == cv2.EVENT_MOUSEMOVE:
        # Conversion des trois couleurs RGB sous la souris en HSV
        px = image[y,x]
        px_array = np.uint8([[px]])
        hsv_px = cv2.cvtColor(px_array,cv2.COLOR_BGR2HSV)
        print(hsv_px)

    if event==cv2.EVENT_MBUTTONDBLCLK:
        color=image[y, x][0]

    if event==cv2.EVENT_LBUTTONDOWN:
        if color>5:
            color-=1

    if event==cv2.EVENT_RBUTTONDOWN:
        if color<250:
            color+=1

    #lo[0]=color-10
    #hi[0]=color+10

# CV Classifier
cascadeClassifier = cv2.CascadeClassifier()

if not cascadeClassifier.load(cv2.samples.findFile("test.xml")):
    print('Impossible de trouver fichier cascade')
    exit(0)

# Compute the estimated position of the bottle with the image processing results
color = 8 # 8Hue value for bright orange 
# We can adjust saturation to include more images
lo = np.array([color - 3, 240, 240]) # Slightly darker and dimmer orange
hi = np.array([color + 3, 255, 255]) # Slighty lighter and brighter orange 

# lo = np.array([color - 8, 220, 220]) # Slightly darker and dimmer orange
#hi = np.array([color + 8, 255, 255]) # Slighty lighter and brighter orange #
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

        frame_gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        frame_gray = cv2.equalizeHist(frame_gray)

        
        image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV) # Convert bgr color scheme to hsv

        pixel_hsv = " ".join(str(values) for values in hsv_px)
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(image, "px HSV: "+pixel_hsv, (10, 260),
               font, 1, (255, 255, 255), 1, cv2.LINE_AA)

        image2 = cv2.erode(image, kernel, iterations=1)
        
        mask = cv2.inRange(image2, lo, hi) # bit-mask selecting desired pixel


        mask = cv2.dilate(mask, kernel, iterations=5)


        # Opencv built-in function to find contours with a mask
        elements = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

        foundArea = Area(minx=0, maxx=0, miny=0, maxy=0)


        if len(elements) > 0:
            print("len higher than 0")
            c = max(elements, key=cv2.contourArea)
            
            x,y,w,h = cv2.boundingRect(c)
            if w > 10 and y < 500: 
                area = Area(minx=x, miny=y, maxx=x+w, maxy=y+h)

                #print(area.sizeX() * area.sizeY())
                if area.sizeX() * area.sizeY() >= 1000:
                    foundArea = area


            
            '''areas = []

            for c in elements:
                x,y,w,h = cv2.boundingRect(c)
                
                if len(areas) == 0:  
                    areas.append(Area(minx=x, miny=y, maxx=x+w, maxy=y+h))

                else:
                    tempArea = Area(minx=x, miny=y, maxx=x+w, maxy=y+h)
                    found = False
                    for i in range(len(areas)):
                        if areas[i].isCommon(tempArea):
                            areas[i] = areas[i].merge(tempArea)
                            found = True
                            break

                    if found == False:
                        areas.append(tempArea)

            
            for area in areas:
                
                if foundArea.isSmaller(area):
                    foundArea = area'''

        
        if foundArea.isEmpty() == False:
            cv2.rectangle(color_image, (foundArea.minx, foundArea.miny), (foundArea.maxx, foundArea.maxy), (0, 0, 255), 2)
            print("Found orange bottle : " + str(foundArea.minx) + " " + str(foundArea.maxx) + " " + str(foundArea.miny) + " " + str(foundArea.maxy))

        
        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback('RealSense', souris)
        cv2.imshow('RealSense', color_image)
        cv2.imshow('mask', mask)

        #image_mask =cv2.bitwise_and(frame, frame, mask= mask)
        
        hist = cv2.calcHist(image, [0], None, [256], [0, 256])
        hist2 = cv2.calcHist(image, [1], None, [256], [0, 256])
        hist3 = cv2.calcHist(image, [2], None, [256], [0, 256])
        pyplot.cla()
        pyplot.plot(hist)
       

        pyplot.plot(hist2)
        

        pyplot.plot(hist3)

        pyplot.savefig('t.png')

        
        cv2.imshow('hist', cv2.imread('t.png'))
        
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
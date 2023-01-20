#!python3

import os
import rclpy
from rclpy.node import Node

import time, numpy as np
import sys, cv2

from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

import pyrealsense2 as rs
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
class Filter():
    def __init__(self, low, high):
        self.low = low
        self.high = high

class ObjectsDetector(Node):

    def __init__(self):
        super().__init__('camera_reader')

        # Config
        self.mergeDistance = 10
        self.baseHue = 8
        self.kernel = np.ones((3, 3), np.uint8)

        self.filters = []

        # We can adjust saturation to include more images
        #self.staticLow = np.array([self.color - 2, 140, 180]) # Slightly darker and dimmer orange
        #self.staticHigh = np.array([self.color + 2, 255, 255]) # Slighty lighter and brighter orange 

        self.staticLow = np.array([0,0,0]) # Slightly darker and dimmer orange
        self.staticHigh = np.array([0,0,0]) # Slighty lighter and brighter orange 

        #self.orangeReference = [6, 150, 180]

        # Creation topic sensor_msgs/image
        #self.image_publisher = self.create_publisher(Image, '/camera/image', 10)
        #self.depth_publisher = self.create_publisher(Image, '/camera/depth', 10)
        #self.infra_publisher1 = self.create_publisher(Image, '/camera/infrared1', 10)
        #self.infra_publisher2 = self.create_publisher(Image, '/camera/infrared2', 10)
        self.create_subscription(Image, '/img', self.onImage, 10)
        self.create_subscription(Image, '/depth', self.onDepth, 10)

 


    def onDepth(self, data: Image):
        bridge = CvBridge()

        #print("imag")

        # convert the ROS Image message to a CV2 Image
        #cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

        try:
            cv_depth_base = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        
            cv_depth = cv2.cvtColor(cv_depth_base, cv2.COLOR_BGR2GRAY)
            
            



            # traitement est storage
            

        except CvBridgeError as e:
            print("CvBridge Error: {0}".format(e))


    def souris(self, event, x, y, flags, param):
    
        


        if event==cv2.EVENT_LBUTTONDOWN:
            #self.orangeReference = self.image_hsv[y, x]

            h = self.image_hsv[y, x][0]
            s = self.image_hsv[y, x][1]
            v = self.image_hsv[y, x][2]
            print("COlor: " + str(h) + ", " + str(s) + ", " + str(v))

            filter = Filter(np.array([h - 2, s - 2, v - 2]), np.array([h + 2, s +   2, v + 2]))

            self.filters.append(filter)
            #self.lo = [self.orangeReference[0] - 5, self.orangeReference[1] - 10, self.orangeReference[2] - 10]
            #self.hi = [self.orangeReference[0] + 5, self.orangeReference[1] + 10, self.orangeReference[2] + 10]

        


    '''
    Traitement image reçue
    '''
    def onImage(self, data: Image):
        bridge = CvBridge()

        #print("imag")

        # convert the ROS Image message to a CV2 Image
        #cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

        try:
            cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        
    
            image_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            self.image_hsv = image_hsv

            image_hsv = cv2.erode(image_hsv, self.kernel, iterations=2)

            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            sift = cv2.SIFT.create()
            kp = sift.detect(gray, None)

            #cv_image = cv2.drawKeypoints(cv_image, kp, cv_image)


            mask = cv2.inRange(image_hsv, self.staticLow, self.staticHigh)

            for filter in self.filters:
                temp_mask = cv2.inRange(image_hsv, filter.low, filter.high)
                mask = cv2.add(mask, temp_mask)
            #mask = cv2.inRange(image_hsv, self.lo, self.hi) # bit-mask selecting desired pixel


            #mask = cv2.add(mask1, mask2)
            #mask = cv2.dilate(mask, self.kernel, iterations=5)

            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.setMouseCallback('RealSense', self.souris)
            
            elements = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

            foundArea = Area(minx=0, maxx=0, miny=0, maxy=0)


            if len(elements) > 0:
                c = max(elements, key=cv2.contourArea)
                x,y,w,h = cv2.boundingRect(c)
                #areas = []

                #print("cc")

                if w > 10 and y < 500:
                    cv2.rectangle(cv_image, (x, y), (x + w,y + h), (0, 0, 255), 2)
                    print("Found orange bottle : " + str(x) + " , " + str(y))



                
                '''for c in elements:
                    x,y,w,h = cv2.boundingRect(c)
                    
                    if w > 10 and y < 500: 
                        continue

                    tempArea = Area(minx=x, miny=y, maxx=x+w, maxy=y+h)

                    if tempArea.sizeX() * tempArea.sizeY() >= 1000:
                        continue

                    if len(areas) == 0:  
                        areas.append(tempArea)

                    else:
                        
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
                cv2.rectangle(cv_image, (foundArea.minx, foundArea.miny), (foundArea.maxx, foundArea.maxy), (0, 0, 255), 2)
                print("Found orange bottle : " + str(foundArea.minx) + " " + str(foundArea.maxx) + " " + str(foundArea.miny) + " " + str(foundArea.maxy))
            cv2.imshow('RealSense', cv_image)
            cv2.imshow('t', image_hsv)
            cv2.imshow('mask', mask)
            cv2.imshow('t2', cv_image)
            cv2.waitKey(1)
            
        
        except CvBridgeError as e:
            print("CvBridge Error: {0}".format(e))

    
        

    


        


        

def main(args=None):
    rclpy.init(args=args)



    objectsDetector = ObjectsDetector()


    


    
        
    rclpy.spin(objectsDetector)

    objectsDetector.pipeline.stop()
        

    objectsDetector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__' :
    main()
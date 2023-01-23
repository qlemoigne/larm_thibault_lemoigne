#!python3

import os
import rclpy
from rclpy.node import Node

import time, numpy as np
import sys, cv2

from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge, CvBridgeError

import pyrealsense2 as rs

import math
DST = 10
import matplotlib.pyplot as plt
from skimage.measure import label, regionprops



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


        # Creation topic sensor_msgs/image

        self.create_subscription(Image, '/img', self.onImage, 10)
        self.create_subscription(Image, '/depth', self.onDepth, 10)

        self.object_publisher = self.create_publisher(String, '/detection', 10)
        

        # Noyau
        self.kernel2 = np.ones((2, 2), np.uint8)
        self.kernel3 = np.ones((3, 3), np.uint8)
        self.kernel4 = np.ones((4, 4), np.uint8)
        self.kernel6 = np.ones((6, 6), np.uint8)


        ####### CRITERES DE CALIBRATION

        # Orange
        #self.orangeFilters = [np.array([13,198,209]),np.array([11,237,194]),np.array([13,147,254]),np.array([12,229,249]),np.array([19,136,254]),]
        #self.orangeLow = np.array([5,33,53])
        #self.orangeHigh = np.array([5,32,53])

        self.orangeFilters = [np.array([15,244,252]),np.array([10,255,246]),np.array([18,209,255]),np.array([24,252,254]),]
        self.orangeLow = np.array([4,35,60])
        self.orangeHigh = np.array([4,35,60])

        '''
Calibration
        self.orangeFilters = [np.array([15,244,252]),np.array([10,255,246]),np.array([18,209,255]),np.array([24,252,254]),]
self.orangeLow = np.array([4,35,60])
self.orangeHigh = np.array([4,35,60])
'''
 


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

            for filter in self.orangeFilters:
                temp_mask = cv2.inRange(image_hsv, filter - self.orangeLow, filter + self.orangeHigh)
                mask = cv2.add(mask, temp_mask)
            
            # Mask contient tout ce qui a été filtré en orangr

            mask = cv2.erode(mask, kernel=self.kernel2, iterations=4)
            mask = cv2.dilate(mask, kernel=self.kernel4, iterations=5)

            mask = cv2.erode(mask, kernel=self.kernel6, iterations=2)
            mask = cv2.dilate(mask, kernel=self.kernel4, iterations=2)

            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            #cv2.setMouseCallback('RealSense', self.souris)
            
            label_image = label(mask)
            regions = regionprops(label_image)
            #print(len(regions))


            for props in regions:
                y0, x0 = props.centroid
                orientation = props.orientation

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


                    cv2.rectangle(cv_image, (int(minc), int(minr)), (int(maxc) ,int(maxr)), (255, 0, 0), 2)
                    print("Bottle orange detected")

                    # Limiter vrai déclanchement à 2 minimum

                    data = String()
                    data.data = "Bouteille orange"
                    self.object_publisher.publish(data)



            
            cv2.imshow('RealSense', cv_image)
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
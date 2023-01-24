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
import matplotlib.pyplot as plt
from skimage.measure import label, regionprops



class ObjectsDetector(Node):

    def __init__(self):
        super().__init__('objects_detector')

        # Filtres du mask initial de la bouteille orange (vide)
        self.staticLow = np.array([0,0,0]) # Slightly darker and dimmer orange
        self.staticHigh = np.array([0,0,0]) # Slighty lighter and brighter orange 


        # Ecoutes des topics liés à la caméra
        self.create_subscription(Image, '/img', self.onImage, 10)
        
        # La profondeur n'est pas utilisé pour le moment
        #self.create_subscription(Image, '/depth', self.onDepth, 10)

        # Création topic émission
        self.object_publisher = self.create_publisher(String, '/detection', 10)
        

        # Noyau pour le nettoyage des masques (Bouteille Orange)
        self.kernel2 = np.ones((2, 2), np.uint8)
        self.kernel3 = np.ones((3, 3), np.uint8)
        self.kernel4 = np.ones((4, 4), np.uint8)
        self.kernel6 = np.ones((6, 6), np.uint8)


        # Critères de calibration (Bouteille orange)
        self.orangeFilters = [np.array([8,166,234]),np.array([12,153,251]),np.array([12,223,209]),np.array([7,152,188]),np.array([14,178,196]),]
        self.orangeLow = np.array([3,31,42])
        self.orangeHigh = np.array([3,31,42])


        # Chargement / resize du template (Bouteille noire)
        self.template = cv2.imread("bouteille-noire.png")
        scale_percent = 30 # percent of original size
        width = int(self.template.shape[1] * scale_percent / 100)
        height = int(self.template.shape[0] * scale_percent / 100)
        dim = (width, height)

        self.template = cv2.resize(self.template, dim, interpolation = cv2.INTER_AREA)
        self.template = cv2.cvtColor(self.template, cv2.COLOR_BGR2GRAY)
        self.template = cv2.Canny(self.template, 50, 200)

        (self.tH, self.tW) = self.template.shape[:2]

        # Détection delay orange
        self.lastDetection = 0
        self.lastDetectionCount = 0

        # Détection delay noir
        self.lastBlackDetection = 0
        self.lastBlackDetectionCount = 0


    def onDepth(self, data: Image):
        bridge = CvBridge()

        try:
            cv_depth_base = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        
            cv_depth = cv2.cvtColor(cv_depth_base, cv2.COLOR_BGR2GRAY)
            
            # analyse profondeur ici
            

        except CvBridgeError as e:
            print("CvBridge Error: {0}".format(e))



    '''
    Traitement image reçue
    '''
    def onImage(self, data: Image):
        bridge = CvBridge()


        try:
            cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        
    
            image_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            self.image_hsv = image_hsv

            image_hsv = cv2.erode(image_hsv, self.kernel3, iterations=2)

            # Détection ORANGE
            mask = cv2.inRange(image_hsv, self.staticLow, self.staticHigh)

            for filter in self.orangeFilters:
                temp_mask = cv2.inRange(image_hsv, filter - self.orangeLow, filter + self.orangeHigh)
                mask = cv2.add(mask, temp_mask)
            
            # Mask contient tout ce qui a été filtré en orange

            mask = cv2.erode(mask, kernel=self.kernel2, iterations=4)
            mask = cv2.dilate(mask, kernel=self.kernel4, iterations=5)

            mask = cv2.erode(mask, kernel=self.kernel6, iterations=2)
            mask = cv2.dilate(mask, kernel=self.kernel4, iterations=2)

            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            label_image = label(mask)
            regions = regionprops(label_image)

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
                    
                    if self.lastDetection + 2 < time.time():
                        self.lastDetectionCount += 1

                        # minimum 3 détection dans la même seconde
                        if self.lastDetectionCount > 3:
                            data = String()
                            data.data = "Bouteille orange"
                            self.object_publisher.publish(data)

                    else:
                        self.lastDetectionCount = 0
                        self.lastDetection = time.time()

            
            # Détéction NOIR

            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            red_image = cv_image[:,:,2]
       
            edged = cv2.Canny(gray_image, 50, 100)
    

    
            result = cv2.matchTemplate(edged, self.template, cv2.TM_CCORR_NORMED)
            (_, maxVal, _, maxLoc) = cv2.minMaxLoc(result)

            if maxVal > 0.19:
                cv2.rectangle(cv_image, (maxLoc[0], maxLoc[1]), (maxLoc[0] + self.tW, maxLoc[1] + self.tH), (0, 255, 0), 2)

                red_image = red_image[maxLoc[1]:maxLoc[1] + self.tH, maxLoc[0]:maxLoc[0] + self.tW]

                count = np.sum(red_image >= 100) -  np.sum(red_image <= 160)

                if count > 100:
                    if self.lastBlackDetection + 2 < time.time():
                        self.lastBlackDetectionCount += 1

                        # minimum 3 détection dans la même seconde
                        if self.lastBlackDetectionCount > 3:
                            data = String()
                            data.data = "Bouteille noire : " + str(count)
                            self.object_publisher.publish(data)

                    else:
                        self.lastBlackDetectionCount = 0
                        self.lastBlackDetection = time.time()

            cv2.imshow('RealSense', cv_image)
            cv2.imshow('mask', mask)
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
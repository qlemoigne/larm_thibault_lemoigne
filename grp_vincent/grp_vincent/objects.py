#!python3

import os
import rclpy
from rclpy.node import Node

import time, numpy as np
import sys, cv2

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
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
        self.create_subscription(Image, '/depth', self.onDepth, 10)

        # Création topic émission
        self.object_publisher = self.create_publisher(String, '/detection', 10)
        
        # COnfig camera
        self.camera_width = 484.0
        self.camera_height = 480.0
        self.hfov = 69

        # Historique de detection
        self.bottlesPoses = []
        self.markerPublisher = self.create_publisher(Marker, '/bottle', 10)

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

    '''
    Retourne la pose estimée
    '''
    def estimatePose(self, x, y, w, h):

        distance = 2000
        for row in self.depth_array[y:y+h, x:x+w]:
            for pixel in row:
                print(pixel)
                if pixel < distance and pixel != 0:
                    distance = pixel  # ros distance with realsense camera
        angle = ((x+w - self.camera_width/2)/(self.camera_width/2))*(self.hfov/2) * math.pi / 180
        estimated_pose = Pose()
        estimated_pose.position.x = distance / 1000 * math.cos(angle) # equals distance * cos(angle from middle of camera)
        estimated_pose.position.y = distance / 1000 * math.sin(angle)  # equals distance * sin(angle from middle of camera)
        return estimated_pose

    '''
    Reception messages liés à la profondeur
    '''
    def onDepth(self, data: Image):
        bridge = CvBridge()

        try:
            cv_depth_base = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        
            #cv_depth_base = cv2.cvtColor(cv_depth_base, cv2.COLOR_BGR2GRAY)
            

            cv2.imshow('cv_depth_base', cv_depth_base)

            #self.depth_array = np.array(cv_depth_base, dtype=np.float32)
            # Analyse profondeur à faire ici
            

        except CvBridgeError as e:
            print("CvBridge Error: {0}".format(e))


    '''
    Traitement d'une bouteille et publication sur topic markers
    '''
    def handleBottle(self, pose, type):

        if self.depth_array:
            print("error not depth found")
            return

        conflict = False

        for existing in self.bottlesPoses:
            if self.areNear(existing, pose):
                conflict = True
                break

        if conflict == False:

            self.bottlesPoses.append(pose)

            data = String()
            data.data = "Bouteille détectée !! " + type
            self.object_publisher.publish(data)
            
            pose_stamped = PoseStamped()
            pose_stamped.pose = pose
            pose_stamped.header.frame_id = "map"
            pose_stamped = self.tfListener.transformPose(
                "map", pose_stamped)
            marker = Marker()
            marker.header.frame_id = "camera_link"
            marker.id = len(self.markers_list)
            print("MARKER ID : " + str(marker.id))
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = pose_stamped.pose
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            self.markerPublisher.publish(marker)

    def areNear(self, existing_marker: Pose, new_marker: Pose):
        a = np.array((existing_marker.position.x, existing_marker.position.y))
        b = np.array((new_marker.position.x, new_marker.position.y))
        dist = np.linalg.norm(a-b)

        print("distance : " + str(dist))
        #print("!!!! DISTANCE : " + str(dist) + " are in same area : " + str(dist<10))
        return dist > 5

    '''
    Reception messages liés à l'image RGB
    '''
    def onImage(self, data: Image):
        bridge = CvBridge()


        try:
            cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        
    
            image_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            self.image_hsv = image_hsv

            # Traitement pour les bouteilles orange

            image_hsv = cv2.erode(image_hsv, self.kernel3, iterations=2)

            
            # Application des mask
            mask = cv2.inRange(image_hsv, self.staticLow, self.staticHigh)

            for filter in self.orangeFilters:
                temp_mask = cv2.inRange(image_hsv, filter - self.orangeLow, filter + self.orangeHigh)
                mask = cv2.add(mask, temp_mask)
            
            # Nettoyage des residus présent dans le mask final

            mask = cv2.erode(mask, kernel=self.kernel2, iterations=4)
            mask = cv2.dilate(mask, kernel=self.kernel4, iterations=5)

            mask = cv2.erode(mask, kernel=self.kernel6, iterations=2)
            mask = cv2.dilate(mask, kernel=self.kernel4, iterations=2)

            # Recherche des regions

            label_image = label(mask)
            regions = regionprops(label_image)

            for props in regions:
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

                    # test perimetre
                    if perimetre > 1000 or perimetre < 90:
                        continue

                    cv2.rectangle(cv_image, (int(minc), int(minr)), (int(maxc) ,int(maxr)), (255, 0, 0), 2)
                    
                    # vérification nombre détections
                    if self.lastDetection + 2 < time.time():
                        self.lastDetectionCount += 1

                        # minimum 3 détection dans les 3s
                        if self.lastDetectionCount > 3:
                            self.handleBottle(self.estimatePose(minc,minr, maxc - minc, maxr - minr), 'orange')


                    else:
                        self.lastDetectionCount = 0
                        self.lastDetection = time.time()

            
            # Traitement bouteilles noires

            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            red_image = cv_image[:,:,2]
       
            edged = cv2.Canny(gray_image, 50, 100)
    
            result = cv2.matchTemplate(edged, self.template, cv2.TM_CCORR_NORMED)
            (_, maxVal, _, maxLoc) = cv2.minMaxLoc(result)

            # seul de correspondance (arbitraire)
            if maxVal > 0.19:
                cv2.rectangle(cv_image, (maxLoc[0], maxLoc[1]), (maxLoc[0] + self.tW, maxLoc[1] + self.tH), (0, 255, 0), 2)

                red_image = red_image[maxLoc[1]:maxLoc[1] + self.tH, maxLoc[0]:maxLoc[0] + self.tW]

                count = np.sum(red_image >= 100) -  np.sum(red_image <= 160)

                # verification du nombre de pixel rouge dans la zone (étiquettes)
                if count > 100:
                    if self.lastBlackDetection + 2 < time.time():
                        self.lastBlackDetectionCount += 1

                        # minimum 3 détection dans les 3s
                        if self.lastBlackDetectionCount > 3:
                            self.handleBottle(self.estimatePose(maxLoc[0], maxLoc[1], self.tW, self.tH), 'black')
                            #data = String()
                            #data.data = "Bouteille noire : " + str(count)
                            #self.object_publisher.publish(data)

                    else:
                        self.lastBlackDetectionCount = 0
                        self.lastBlackDetection = time.time()

            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
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
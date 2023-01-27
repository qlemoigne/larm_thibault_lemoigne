#!python3

import os
import rclpy
from rclpy.node import Node

import time, numpy as np
import sys, cv2
import tf2_ros
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose

from visualization_msgs.msg import Marker
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge, CvBridgeError

import pyrealsense2 as rs

import math
import matplotlib.pyplot as plt
from skimage.measure import label, regionprops
from geometry_msgs.msg import PoseStamped

import PyKDL

class ObjectsDetector(Node):

    def __init__(self):
        super().__init__('objects_detector')

        # Filtres du mask initial de la bouteille orange (vide)
        self.staticLow = np.array([0,0,0]) # Slightly darker and dimmer orange
        self.staticHigh = np.array([0,0,0]) # Slighty lighter and brighter orange 

        # Création topic émission
        self.object_publisher = self.create_publisher(String, '/detection', 10)
        
        # Listeners TF
        # Transform tool:
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Historique de detection
        self.bottlesPoses = []
        self.markerPublisher = self.create_publisher(Marker, '/bottle', 10)

        # Noyau pour le nettoyage des masques (Bouteille Orange)
        self.kernel2 = np.ones((2, 2), np.uint8)
        self.kernel3 = np.ones((3, 3), np.uint8)
        self.kernel4 = np.ones((4, 4), np.uint8)
        self.kernel6 = np.ones((6, 6), np.uint8)

        # Critères de calibration (Bouteille orange) Définis par le fichier calibrer.py
        self.orangeFilters = [np.array([12,222,249]),np.array([10,255,180]),np.array([14,239,217]),np.array([12,248,190]),np.array([13,252,198]),np.array([14,236,232]),np.array([15,214,253]),np.array([13,233,253]),np.array([12,209,249]),np.array([7,162,255]),np.array([9,177,252]),np.array([5,207,223]),np.array([6,137,252]),np.array([9,217,213]),np.array([9,130,253]),np.array([13,195,250]),np.array([11,183,209]),np.array([6,211,191]),np.array([13,154,255]),np.array([7,229,150]),np.array([13,233,175]),np.array([12,187,200]),]
        self.orangeLow = np.array([2,16,32])
        self.orangeHigh = np.array([2,17,32])


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
        
        # Creation topic sensor_msgs/image
        self.image_publisher = self.create_publisher(Image, '/camera/image', 10)
        self.depth_publisher = self.create_publisher(Image, '/camera/depth', 10)

        # Get device product line for setting a supporting resolution
        self.cvBridge = CvBridge();


        # Lancement camera
        self.pipeline = rs.pipeline()
        self.config = rs.config()


        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = self.config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        print( f"Connect: {device_product_line}" )
        found_rgb = True
        for s in device.sensors:
            print( "Name:" + s.get_info(rs.camera_info.name) )
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True

        if not (found_rgb):
            print("RGB camera equired !!!")
            exit(0)

        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 60)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)
        

        self.profile = self.pipeline.start(self.config)
        
        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()

        # Config distance
        self.clipping_distance_in_meters = 4
        self.clipping_distance = self.clipping_distance_in_meters / self.depth_scale
        
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
        
        self.timer = self.create_timer(0.05, self.cameraHandler) # 0.1 seconds to target a frequency of 10 hertz

    '''
    Traitement frame reçues
    '''
    def cameraHandler(self):
    
        frames = self.pipeline.wait_for_frames()

        # Align the depth frame to color frame
        aligned_frames = self.align.process(frames)
        
        self.aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()
        
        # Validate that both frames are valid
        if not self.aligned_depth_frame or not color_frame:
            return
            
            
        self.depth_image = np.asanyarray(self.aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        self.depth_intrin = self.aligned_depth_frame.profile.as_video_stream_profile().intrinsics
            
        # Remove background - Set pixels further than clipping_distance to grey
        grey_color = 153
        depth_image_3d = np.dstack((self.depth_image,self.depth_image,self.depth_image)) #depth image is 1 channel, color is 3 channels
        bg_removed = np.where((depth_image_3d > self.clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)    
        
        # Appel listeners
        self.onImage(bg_removed)

        # Envoit dans les topics
        msg_image = self.cvBridge.cv2_to_imgmsg(bg_removed,"bgr8")
        msg_image.header.stamp = self.get_clock().now().to_msg()
        msg_image.header.frame_id = "image"
        self.image_publisher.publish(msg_image)


    def transform_to_kdl(self, t):
     return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                                  t.transform.rotation.z, t.transform.rotation.w),
                        PyKDL.Vector(t.transform.translation.x, 
                                     t.transform.translation.y, 
                                     t.transform.translation.z))

    def do_transform_pose(self, pose, transform):
        f = self.transform_to_kdl(transform) * PyKDL.Frame(PyKDL.Rotation.Quaternion(pose.pose.orientation.x, pose.pose.orientation.y,
                                                                            pose.pose.orientation.z, pose.pose.orientation.w),
                                                    PyKDL.Vector(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z))
        
        res = PoseStamped()
        res.pose.position.x = f.p[0]
        res.pose.position.y = f.p[1]
        res.pose.position.z = f.p[2]
        (res.pose.orientation.x, res.pose.orientation.y, res.pose.orientation.z, res.pose.orientation.w) = f.M.GetQuaternion()
        res.header = transform.header
        return res

    '''
    Retourne la pose / position estimée dans le répére map
    '''
    def estimatePose(self, x, y, w, h):

        # Point central
        mx = x + w/2
        my = y + h/2
        
        # Profondeur avec camera
        depth2 = self.aligned_depth_frame.get_distance(int(mx), int(my))
        depth_point = rs.rs2_deproject_pixel_to_point(self.depth_intrin, [mx, my], depth2)

        currentTime= rclpy.time.Time()

        # POse dans le repére camera_link
        pose = Pose()
        pose.position.x = depth_point[2]
        pose.position.y = -depth_point[0]
        pose.position.z = depth_point[1]

        # demande de transformation de camera_link à map
        stampedTransform = None

        try:
            stampedTransform= self.tf_buffer.lookup_transform(
                        'map',
                        'camera_link',
                        currentTime)

        except tf2_ros.TransformException as tex:
            self.get_logger().info( f'Could not transform the Pose from ": {tex}')
            return

        # Translation de la Pose dans le repère map
        stampedGoal= PoseStamped()
        stampedGoal.pose= pose
        stampedGoal.header.frame_id= 'map'
        stampedGoal.header.stamp= self.get_clock().now().to_msg()
        
        globalPose = self.do_transform_pose( stampedGoal, stampedTransform )

        return globalPose

    '''
    Traitement d'une bouteille et publication sur topic markers
    '''
    def handleBottle(self, pose, bottleType):

        conflict = False

        # On regarde si deux point sont proches
        for existing in self.bottlesPoses:
            if self.areNear(existing, pose.pose):
                conflict = True
                break

        # Pas de conflict on ajoute
        if conflict == False:

            self.bottlesPoses.append(pose.pose)

            # filtre de hauteur
            if pose.pose.position.y < -1.5:
                return;

            # filtre plus restrictif pour les bouteilels noir (en pratique n'est pas utilisé car on ne crée par de markers pour les bouteilles noires)
            if bottleType == "black":
                if pose.pose.position.y < -0.5 or pose.pose.position.y >= 1.5:
                    return;

            # Envoit message
            data = String()
            data.data = "Bouteille détectée !! " + bottleType + " pose : " + str(pose.pose.position.y)
            self.object_publisher.publish(data)
            
            # Envoit marker
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "my_namespace"
            marker.id = len(self.bottlesPoses)
            marker.type = Marker.CYLINDER
            marker.action =Marker.ADD

            marker.pose = pose.pose
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.3
            marker.color.a = 1.0
            
            if bottleType == "orange":
                marker.color.r = 1.0
                marker.color.g = 0.5
                marker.color.b = 0.0
            else:
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 0.0

            self.markerPublisher.publish(marker)

    '''
    Permet de savoir si 2 poses sont proche (soit à moins de 90cm de distance)
    '''
    def areNear(self, existing_marker: Pose, new_marker: Pose):

        # Vérification validité de marker
        if new_marker == None or existing_marker == None:
            return True

        a = np.array((existing_marker.position.x, existing_marker.position.y))
        b = np.array((new_marker.position.x, new_marker.position.y))
        dist = np.linalg.norm(a-b)

        return dist < 0.9

    '''
    Reception messages liés à l'image RGB
    '''
    def onImage(self, cv_image):
        try:
            image_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            self.image_hsv = image_hsv

            # Traitement pour les bouteilles orange
            image_hsv = cv2.erode(image_hsv, self.kernel3, iterations=2)

            # Application des mask pour ne garder que le orange calibré
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

            # Analyse des regions
            for props in regions:
                minr, minc, maxr, maxc = props.bbox

                rsize = maxr - minr
                csize = maxc - minc

                # Calcul ratio
                ratio = csize / rsize

                # On restreint aux bouteilles verticales
                if csize > rsize:
                    continue
                
                if props.extent > 0.85 or props.extent < 0.35:
                    continue

                # Restriction du ratio

                if ratio <= 0.5 and ratio >= 0.3:
                    perimetre = 2 * rsize + 2 * csize

                    # test perimetre
                    if perimetre > 1000 or perimetre < 90:
                        continue

                    # On affiche rectangle pour debug
                    cv2.rectangle(cv_image, (int(minc), int(minr)), (int(maxc) ,int(maxr)), (255, 0, 0), 2)
                    
                    # vérification nombre détections : au moins 4 en 2s pour être valide
                    if self.lastDetection + 2 < time.time():
                        self.lastDetectionCount += 1

                        if self.lastDetectionCount > 4:

                            # Calcul de la position dans frame map
                            pose = self.estimatePose(minc, minr, maxc - minc, maxr - minr)

                            # Si valide on envoit marker
                            if pose != None:
                                self.handleBottle(pose, 'orange')


                    else:
                        self.lastDetectionCount = 0
                        self.lastDetection = time.time()

            
            # Traitement bouteilles noires

            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # On extrait le canal rouge
            red_image = cv_image[:,:,2]

            # Filtre de canny permet extraire des "zones"
            edged = cv2.Canny(gray_image, 50, 100)

            # On essaie de détecter notre template (bouteille noire avec étiquette rouge)
            result = cv2.matchTemplate(edged, self.template, cv2.TM_CCORR_NORMED)
            (_, maxVal, _, maxLoc) = cv2.minMaxLoc(result)

            # seul de correspondance (arbitraire obtenu avec les tests) 0.20
            if maxVal > 0.20:
                cv2.rectangle(cv_image, (maxLoc[0], maxLoc[1]), (maxLoc[0] + self.tW, maxLoc[1] + self.tH), (0, 255, 0), 2)

                red_image = red_image[maxLoc[1]:maxLoc[1] + self.tH, maxLoc[0]:maxLoc[0] + self.tW]

                count = np.sum(red_image >= 100) -  np.sum(red_image <= 160)

                # verification du nombre de pixel rouge dans la zone (étiquettes)
                if count > 100:
                    # Vérification nombre détectins : au moins 3 en 2s pour être valide
                    if self.lastBlackDetection + 2 < time.time():
                        self.lastBlackDetectionCount += 1

                        # minimum 3 détection dans les 2s
                        if self.lastBlackDetectionCount > 3:

                            # n'ajoute pas de marker pour les bouteilles noir (car trop de faux positifs), mais envoit un msg
                            #pose = self.estimatePose(maxLoc[0], maxLoc[1], self.tW, self.tH)

                            #if pose != None:
                                #self.handleBottle(pose, 'black')
                            data = String()
                            data.data = "Bouteille noire"
                            self.object_publisher.publish(data)

                    else:
                        self.lastBlackDetectionCount = 0
                        self.lastBlackDetection = time.time()

            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', cv_image)
            cv2.imshow('Mask', mask)

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
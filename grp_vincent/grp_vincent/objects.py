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


        # Ecoutes des topics liés à la caméra
        #self.create_subscription(Image, '/img', self.onImage, 10)
        
        # La profondeur n'est pas utilisé pour le moment
        #self.create_subscription(Image, '/depth', self.onDepth, 10)

        # Création topic émission
        self.object_publisher = self.create_publisher(String, '/detection', 10)
        
        # Listeners TF
        # Transform tool:
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
 

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
        self.orangeFilters = [np.array([10,202,255]),np.array([11,255,234]),np.array([9,251,250]),np.array([12,243,250]),np.array([16,189,254]),np.array([14,255,249]),np.array([19,231,250]),]
        self.orangeLow = np.array([2,35,37])
        self.orangeHigh = np.array([3,22,37])



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


        # Lancement camera
        self.pipeline = rs.pipeline()
        config = rs.config()

        # Creation topic sensor_msgs/image
        self.image_publisher = self.create_publisher(Image, '/camera/image', 10)
        self.depth_publisher = self.create_publisher(Image, '/camera/depth', 10)

        # Get device product line for setting a supporting resolution
        self.cvBridge = CvBridge();

        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
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

        self.pipeline.start(config)
        self.timer = self.create_timer(0.05, self.cameraHandler) # 0.1 seconds to target a frequency of 10 hertz

    '''
    Traitement frame reçues
    '''
    def cameraHandler(self):
        frames = self.pipeline.wait_for_frames()

        #depth_frame = frames.first(rs.stream.depth)
        color_frame = frames.first(rs.stream.color)

        color_image = np.asanyarray(color_frame.get_data())

        self.color_intrin = color_frame.profile.as_video_stream_profile().intrinsics

        self.depth_frame = frames.get_depth_frame()

        if not (self.depth_frame and color_frame):
            return

    
        # Appel listeners
        self.onImage(color_image)

        # Envoit dans les topics
        msg_image = self.cvBridge.cv2_to_imgmsg(color_image,"bgr8")
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
    Retourne la pose estimée
    '''
    def estimatePose(self, x, y, w, h):


        mx = x + w/2
        my = y + h/2

        depth = self.depth_frame.get_distance(int(mx), int(my))

        #if depth < 0.2 or depth > 3.0:
        #    return

        print("depth = " + str(depth))
        dx ,dy, dz = rs.rs2_deproject_pixel_to_point(self.color_intrin, [int(mx),int(my)], depth)
        distance = math.sqrt(((dx)**2) + ((dy)**2) + ((dz)**2))

        #print("d = " + str(distance) + " x = " + str(dx) + " y = " + str(dy) + " z = " + str(dz))

        currentTime= rclpy.time.Time()

        pose = Pose()
        pose.position.x = dx
        pose.position.y = dy
        pose.position.z = dz

        #print(pose)
        
        # Get Transformation
        '''stampedTransform = None

        try:
            stampedTransform= self.tf_buffer.lookup_transform(
                        'map',
                        'camera_link',
                        currentTime)


            
        except tf2_ros.TransformException as tex:
            self.get_logger().info( f'Could not transform the Pose from ": {tex}')
            return'''

  
        # Compute goal in local coordinates
        '''stampedGoal= PoseStamped()
        stampedGoal.pose= pose
        stampedGoal.header.frame_id= 'camera_link'
        stampedGoal.header.stamp= self.get_clock().now().to_msg()
        
        globalPose = self.do_transform_pose( stampedGoal, stampedTransform )
        return globalPose'''
        return pose
        

    '''
    Traitement d'une bouteille et publication sur topic markers
    '''
    def handleBottle(self, pose, type):

        conflict = False

        for existing in self.bottlesPoses:
            if self.areNear(existing, pose):
                conflict = True
                break

        if conflict == False:

            #self.bottlesPoses.append(pose)

            data = String()
            data.data = "Bouteille détectée !! " + type
            self.object_publisher.publish(data)
            

            print("send marker")

            marker = Marker()
            marker.header.frame_id = "camera_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "my_namespace"
            marker.id = len(self.bottlesPoses)
            marker.type = Marker.CYLINDER
            marker.action =Marker.ADD
            


            marker.pose = pose
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.4
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            self.markerPublisher.publish(marker)

    def areNear(self, existing_marker: Pose, new_marker: Pose):

        if new_marker == None:
            print("invalid new marker")
            return True

        a = np.array((existing_marker.position.x, existing_marker.position.y))
        b = np.array((new_marker.position.x, new_marker.position.y))
        dist = np.linalg.norm(a-b)

        
        #print("!!!! DISTANCE : " + str(dist) + " are in same area : " + str(dist<10))
        return dist > 5

    '''
    Reception messages liés à l'image RGB
    '''
    def onImage(self, cv_image):
        #bridge = CvBridge()


        try:
            #cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        
    
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

                            pose = self.estimatePose(minc,minr, maxc - minc, maxr - minr)

                            if pose != None:
                                self.handleBottle(pose, 'orange')


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


                            pose = self.estimatePose(maxLoc[0], maxLoc[1], self.tW, self.tH)

                            if pose != None:
                                self.handleBottle(pose, 'black')
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
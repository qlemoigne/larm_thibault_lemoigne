#!/usr/bin/env python3
## Doc: https://dev.intelrealsense.com/docs/python2

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import signal, time, numpy as np
import cv2, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Realsense Node:
class Realsense(Node):
    def __init__(self):
        super().__init__('realsense')

        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.bridge=CvBridge()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper( self.pipeline )
        device = self.config.resolve(pipeline_wrapper).get_device()

        self.sensors= []
        print( f"Connect: { str(device.get_info(rs.camera_info.product_line))}" )
        for s in device.sensors:
            info= s.get_info(rs.camera_info.name)
            print( "Name: " + info )
            self.sensors.append( info )

    def connect_imgs(self, fps= 60):
        if ("Stereo Module" not in self.sensors) or ("RGB Camera" not in self.sensors) :
            exit(0)
        
        # prepare publisher:
        self.img_pub= self.create_publisher( Image, "img", 10)
        self.depth_pub= self.create_publisher( Image, "depth", 10)

        # enable stream:
        self.config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, fps)
        self.config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, fps)

        # Start streaming
        self.pipeline.start(self.config)

        # Align
        self.align_to = rs.stream.depth
        self.align = rs.align(self.align_to)

    def read_imgs(self):
        # Wait for a coherent tuple of frames: depth, color and accel
        frames = self.pipeline.wait_for_frames()

        #Aligning color frame to depth frame
        aligned_frames =  self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        aligned_color_frame = aligned_frames.get_color_frame()

        #color_frame = frames.first(rs.stream.color)
        depth_frame = frames.first(rs.stream.depth)
        
        # Convert images to numpy arrays
        self.depth_image = np.asanyarray(depth_frame.get_data())
        self.color_image = np.asanyarray(aligned_color_frame.get_data())

    def publish_imgs(self):
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_image, alpha=0.03), cv2.COLORMAP_JET)

        msg_image = self.bridge.cv2_to_imgmsg( self.color_image,"bgr8" )
        msg_image.header.stamp = self.get_clock().now().to_msg()
        msg_image.header.frame_id = "camera_link"
        self.img_pub.publish(msg_image)

        msg_depth = self.bridge.cv2_to_imgmsg(depth_colormap,"bgr8")
        msg_depth.header.stamp = msg_image.header.stamp
        msg_depth.header.frame_id = "camera_link"
        self.depth_pub.publish(msg_depth)

    def show_imgs(self):
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Show images
        images = np.hstack(( self.color_image, depth_colormap )) # supose that depth_colormap_dim == color_colormap_dim (640x480) otherwize: resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        cv2.waitKey(1)

# Catch Interuption signal:
isOk= True

def signalInteruption(signum, frame):
    global isOk
    print( "\nCtrl-c pressed" )
    isOk= False

signal.signal(signal.SIGINT, signalInteruption)

# Node processes:
def process_img(args=None):
    rclpy.init(args=args)
    rsNode= Realsense()
    rsNode.connect_imgs()
    while isOk:
        rsNode.read_imgs()
        rsNode.publish_imgs()
        rclpy.spin_once(rsNode, timeout_sec=0.01)
    # Stop streaming
    print("Ending...")
    rsNode.pipeline.stop()
    # Clean end
    rsNode.destroy_node()
    rclpy.shutdown()

# script execution:
if __name__ == '__main__' :
    process_img()

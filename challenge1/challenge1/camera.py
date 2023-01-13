#!python3

import rclpy
from rclpy.node import Node


import pyrealsense2 as rs
import time, numpy as np
import sys, cv2

from sensor_msgs.msg import Image

from cv_bridge import CvBridge

class CameraReader(Node):

    def __init__(self):
        super().__init__('camera_reader')

        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()

        # Get device product line for setting a supporting resolution
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

        config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
        #config.enable_stream(rs.stream.infrared, 1, 848, 480, rs.format.y8, 30)
        #config.enable_stream(rs.stream.infrared, 2, 848, 480, rs.format.y8, 30)

        # Start streaming
        self.pipeline.start(config)


        self.count= 1
        self.refTime= time.process_time()
        self.freq= 10

        # Creation topic sensor_msgs/image
        self.image_publisher = self.create_publisher(Image, '/camera/image', 10)
        self.depth_publisher = self.create_publisher(Image, '/camera/depth', 10)
        #self.infra_publisher1 = self.create_publisher(Image, '/camera/infrared1', 10)
        #self.infra_publisher2 = self.create_publisher(Image, '/camera/infrared2', 10)


        self.cvBridge = CvBridge()

        # Run a 20fps
        self.timer = self.create_timer(0.05, self.parseCamera) 

        


    def parseCamera(self):
        frames = self.pipeline.wait_for_frames()

        depth_frame = frames.first(rs.stream.depth)
        color_frame = frames.first(rs.stream.color)

        
        #infra_frame_1 = frames.get_infrared_frame(1)
        #infra_frame_2 = frames.get_infrared_frame(2)
    
        #infra_image_1 = np.asanyarray(infra_frame_1.get_data())
        #infra_image_2 = np.asanyarray(infra_frame_2.get_data())


        if not (depth_frame and color_frame):
            return
            
        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())



        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        #infra_colormap_1 = cv2.applyColorMap(cv2.convertScaleAbs(infra_image_1, alpha=1), cv2.COLORMAP_JET)
		
	    # Utilisation de colormap sur l'image infrared de la Realsense (image convertie en 8-bit par pixel)
        #infra_colormap_2 = cv2.applyColorMap(cv2.convertScaleAbs(infra_image_2, alpha=1), cv2.COLORMAP_JET)	


        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        sys.stdout.write( f"\r- {color_colormap_dim} - {depth_colormap_dim} - ({round(self.freq)} fps)" )
            
        #print(f"- {color_colormap_dim} - {depth_colormap_dim} - ({round(self.freq)} fps)")

        # Show images
        images = np.hstack((color_image, depth_colormap)) # supose that depth_colormap_dim == color_colormap_dim (640x480) otherwize: resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)

        # Show images
        #cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        #cv2.imshow('RealSense', images)
        #cv2.waitKey(1)

        # Send to topic

        msg_image = self.cvBridge.cv2_to_imgmsg(color_image,"bgr8")
        msg_image.header.stamp = self.get_clock().now().to_msg()
        msg_image.header.frame_id = "image"
        self.image_publisher.publish(msg_image)


        msg_depth = self.cvBridge.cv2_to_imgmsg(depth_colormap,"bgr8")
        msg_depth.header.stamp = msg_image.header.stamp
        msg_depth.header.frame_id = "depth"
        self.depth_publisher.publish(msg_depth)

        #msg_infra = self.cvBridge.cv2_to_imgmsg(infra_colormap_1,"bgr8")
        #msg_infra.header.stamp = msg_image.header.stamp
        #msg_infra.header.frame_id = "infrared_1"
        #self.infra_publisher1.publish(msg_infra)

        #msg_infra = self.cvBridge.cv2_to_imgmsg(infra_colormap_2,"bgr8")
        #msg_infra.header.stamp = msg_image.header.stamp
        #msg_infra.header.frame_id = "infrared_2"
        #self.infra_publisher2.publish(msg_infra)

            
        # Frequency:
        if self.count == 10 :
            newTime= time.process_time()
            self.freq= 10/((newTime-self.refTime))
            self.refTime= newTime
            self.count= 0
        self.count+= 1

        

    


        


        

def main(args=None):
    rclpy.init(args=args)



    cameraReader = CameraReader()


    


    
        
    rclpy.spin(cameraReader)

    cameraReader.pipeline.stop()
        

    cameraReader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__' :
    main()
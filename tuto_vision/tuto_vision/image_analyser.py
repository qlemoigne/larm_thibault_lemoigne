#!python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image


class CameraReader(Node):

    def __init__(self):
        super().__init__('camera_analyser')

        

        # Creation topic sensor_msgs/image
        self.create_subscription(Image, '/camera/image', self.cameraImageCallback, 10)
        
        self.create_subscription(Image, '/camera/depth', self.cameraDepthCallback, 10)

        self.lastImage = Image()
        self.lastDepth = Image()



        # Run a 20fps
        self.timer = self.create_timer(0.01, self.analyse) 

        

    def cameraImageCallback(self, data):
        self.lastImage = data


    def cameraDepthCallback(self, data):
        self.lastDepth = data


    def analyse(self):
        print("Analysing ...")

        

        

    


        


        

def main(args=None):
    rclpy.init(args=args)



    cameraReader = CameraReader()


    


    
        
    rclpy.spin(cameraReader)

    cameraReader.pipeline.stop()
        

    cameraReader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__' :
    main()
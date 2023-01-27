#!python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud


class ScanInterpret(Node):

    def __init__(self):
        super().__init__('scan_interpreter')
        self.create_subscription( LaserScan, 'scan', self.scan_callback, 10)

        print("Scan echo launched...")

        self.scanner_publisher = self.create_publisher(PointCloud, '/laser/pointcloud', 10)

    def scan_callback(self, scanMsg):

        obstacles= []
        angle= scanMsg.angle_min

        # x : Avant arrière, > 0 devant robot
        # Y : gauche / droite

        for aDistance in scanMsg.ranges :
            # Si point trop loin on ignore
            if aDistance < 5.0 :

                p = Point32()

                p.x = math.cos(angle) * aDistance
                p.y = math.sin( angle ) * aDistance
                p.z = 0.0
                
                # si point derrière on ignore
                if p.x > 0:
                    obstacles.append(p)
            angle+= scanMsg.angle_increment

        
        pc = PointCloud()

        pc.header = scanMsg.header
        pc.points = obstacles

        self.scanner_publisher.publish(pc)

def main(args=None):
    rclpy.init(args=args)
    scanInterpret = ScanInterpret()
    rclpy.spin(scanInterpret)
    scanInterpret.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__' :
    main()
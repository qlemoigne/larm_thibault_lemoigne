from asyncio import Future
from math import sqrt
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from sensor_msgs.msg import PointCloud


import sys
sys.path.append("/install/kobuki_ros_interfaces/lib/python3.8/site-packages")
from kobuki_ros_interfaces.msg import Sound
from kobuki_ros_interfaces.msg import Led
from kobuki_ros_interfaces.msg import WheelDropEvent

import time
import random
OBS_DIST = 0.30

class FuturNode(Node):

    def __init__(self, name):
        super().__init__(name)

        self.future = Future()

    def finish(self):
        self.future.set_result('ok')


class ObstacleAreaData():

    def __init__(self, name, minx, miny, maxx, maxy):

        self.name = name
        self.count = 0
        self.closestDistance = 10000.0

        self.countCopy = 0
        self.closestDistanceCopy = 10000.0

        self.minx = minx
        self.miny = miny
        self.maxx = maxx
        self.maxy = maxy

        self.projectedDist = False

    def update(self):
        self.count = self.countCopy
        self.closestDistance = self.closestDistanceCopy

        self.countCopy = 0
        self.closestDistanceCopy = 10000.0

    '''
    Il y a un objet dans la salle
    '''
    def blocked(self):
        return self.closestDistance < 10000.0

    '''
    Indique si un point se trouve dans la zone
    '''
    def isInside(self, point):
        return point.x >= self.minx and point.y >= self.miny and point.x <= self.maxx and point.y <= self.maxy


    def debug(self):
        print(f"State : " + self.name + " => C = " + str(self.count) + " CLOSEST = " + str(self.closestDistance) + ", BLOCK = " + str(self.blocked()) + "")

    def computePoint(self, point):
        if self.isInside(point):
            
            if self.projectedDist == False:
                norme = point.x * point.x + point.y * point.y

                distance = sqrt(norme)

                # ignore pts parasite
                if distance < 0.02:
                    return
            else:
                distance = point.x

            self.countCopy += 1

            if distance < self.closestDistanceCopy:
                self.closestDistanceCopy = distance


class MoveNode(FuturNode):

    def __init__(self):
        super().__init__('move')
        self.velocity_publisher = self.create_publisher(Twist, '/multi/cmd_nav', 10)

        self.sound_publisher = self.create_publisher(Sound, '/commands/sound', 10)

        self.led1_publisher = self.create_publisher(Led, '/commands/led1', 10)

        self.led2_publisher = self.create_publisher(Led, '/commands/led2', 10)


        self.create_subscription(PointCloud, '/laser/pointcloud', self.scan_callback, 10)
        self.create_subscription(WheelDropEvent, '/events/wheel_drop', self.wheel_callback, 10)

        self.iterations = 0
        self.timer = self.create_timer(0.1, self.activate) # 0.1 seconds to target a frequency of 10 hertz

        self.frontLeft = ObstacleAreaData(name="Front Left", miny=0.0, maxy=0.19, minx=0.05, maxx=OBS_DIST)
        self.frontRight = ObstacleAreaData(name="Front Right", miny=-0.19, maxy=0.0, minx=0.05, maxx=OBS_DIST)

        self.frontExtremeLeft = ObstacleAreaData(name="Front extreme Left", miny=0.19, maxy=0.4, minx=0.05, maxx=OBS_DIST)
        self.frontExtremeRight = ObstacleAreaData(name="Front extreme Right", miny=-0.4, maxy=-0.19, minx=0.05, maxx=OBS_DIST)

       

        self.frontPath = ObstacleAreaData(name="Front path", miny=-0.16, maxy=0.16 ,minx=0.1, maxx= 0.3)


        # 0.3 m
        self.longObstacle = ObstacleAreaData(name="Long obs", miny=-0.2, maxy=0.2,minx=OBS_DIST, maxx=OBS_DIST + 0.4)
        

        self.longFrontLeft = ObstacleAreaData(name="Long front left obs", miny=0.0, maxy=0.25,minx=OBS_DIST, maxx=OBS_DIST + 0.4)
        self.longFrontRight = ObstacleAreaData(name="Long front right obs", miny=-0.25, maxy=0.0,minx=OBS_DIST, maxx=OBS_DIST + 0.4)
        self.longFrontLeft.projectedDist = True
        self.longFrontRight.projectedDist = True

        
        self.alternateDir = False

        self.longRangeObs = False
        self.xSpeed = 0.0

        self.drift = 0.0

        self.rotateDir = 0.0
        self.rotateCount = 0

        self.blockTime = 0

        self.lastAlternateDirChange = 0
        self.driftDelay = 0
        self.droped = False


    def wheel_callback(self, data):

        if data.state == 1:
            self.droped = True
        else: self.droped = False

    def isFinish(self):
        return self.iterations > 20000

    def scan_callback(self, pc):
        tempDrift = 0.0

        for point in pc.points:
            self.frontLeft.computePoint(point)
            self.frontRight.computePoint(point)


            self.frontExtremeLeft.computePoint(point)
            self.frontExtremeRight.computePoint(point)
            self.longObstacle.computePoint(point)

            self.frontPath.computePoint(point)

            self.longFrontLeft.computePoint(point)
            self.longFrontRight.computePoint(point)

        self.frontLeft.update()
        self.frontRight.update()
        self.frontExtremeLeft.update()
        self.frontExtremeRight.update()
        self.longObstacle.update()
        self.frontPath.update()

        led0 = 0
        led1 = 0

        if self.longFrontLeft.blocked():
            led0 = 2

        if self.longFrontRight.blocked():
            led1 = 2

        if self.frontLeft.blocked():
            led0 = 3

        if self.frontRight.blocked():
            led1 = 3

        s = Led()
        s.value = led0
        self.led1_publisher.publish(s)

        s.value = led1
        self.led2_publisher.publish(s)

        self.longFrontLeft.update()
        self.longFrontRight.update()



        print("--------------------------------- BEGIN EXEC -----------------------")
        self.frontLeft.debug()
        self.frontRight.debug()
        self.frontExtremeLeft.debug()
        self.frontExtremeRight.debug()
        self.longFrontLeft.debug()
        self.longFrontRight.debug()
        self.frontPath.debug()


        # Détection blockage robot

        if self.frontLeft.blocked() == False and self.frontRight.blocked() >= False:
            self.blockTime = 0
        else:
            self.blockTime += 1
            print("Block time : " + str(self.blockTime))

    def activate(self):
        
        if self.droped == True:
            print("droped")
            s = Sound()
            s.value = 1
            self.sound_publisher.publish(s)
            return;

        velo = Twist()


        if self.driftDelay > 0:
            self.driftDelay -= 1

        if self.blockTime > 5:

        
            s = Sound()
            s.value = 5
            self.sound_publisher.publish(s)
            
            velo.linear.x = 0.01

            self.rotateCount = 20
            self.blockTime = 0
        
            if self.rotateDir == 0.0:
                if random.randint(0, 1) == 0:
                    self.rotateDir = 1.5
                else:
                    self.rotateDir = -1.5


        # Requete rotation non finie
        if self.rotateCount > 0:
            velo.linear.x = 0.01
            velo.angular.z = self.rotateDir

            self.rotateCount -= 1

            print("Rotate mode")

            if self.frontPath.blocked() == False:
                velo.linear.x = 0.2
                self.rotateCount = 0
                velo.angular.z = 0.0
                print("Unlocking rotate because front path free")

            self.velocity_publisher.publish(velo)
            return


        # Mur en face (proche)
        if self.frontLeft.blocked() == False and self.frontRight.blocked() == False:   

            if self.xSpeed == 0:
                self.xSpeed = 0.03

            if self.longObstacle.blocked() == False:

                if self.xSpeed < 0.5:
                    self.xSpeed += 0.03
            
            
                
            else:
                if self.xSpeed > 0.3:
                    self.xSpeed -= 0.05
                else:
                    if self.xSpeed + 0.05 <= 0.3:
                        self.xSpeed += 0.05
           
           
            velo.linear.x = self.xSpeed

            # Compute drift

            if self.driftDelay <= 0:



                if ((self.longFrontLeft.count > 0 and self.longFrontLeft.closestDistance < self.longFrontRight.closestDistance) or self.frontExtremeLeft.blocked()) and self.frontExtremeRight.blocked() == False:
                    self.drift = -0.45 * 1 / self.longFrontLeft.closestDistance
                    print("Drift vers Droite " + str(self.longFrontLeft.closestDistance))

                    if self.frontExtremeLeft.blocked():
                        self.drift -= 0.1

                    self.driftDelay = 5

                   

                # obstacle lointaint à droite (donc on veut drifter à gauche) si il n'y a rien a proximité gauche
                elif ((self.longFrontRight.count > 0 and self.longFrontRight.closestDistance < self.longFrontLeft.closestDistance) or self.frontExtremeRight.blocked()) and self.frontExtremeLeft.blocked() == False:
                    self.drift = 0.45 * 1 / self.longFrontRight.closestDistance
                    print("Drift vers Gauche " + str(self.longFrontRight.closestDistance))
                
                    if self.frontExtremeRight.blocked():
                        self.drift += 0.1

                    self.driftDelay = 5

                else:
                    self.drift = 0.0
            else:

                # on applique drift precedent si tjr valide

                # drift vers droite et droite bloqué OU plus l'obs, on annule
                if self.drift < 0 and (self.frontExtremeRight.blocked() or self.longFrontLeft.blocked() == False):
                    self.drift = 0.0

                # drift vers gauche et gauche bloqué Ou plus l'obs, on annule
                if self.drift > 0 and (self.frontExtremeLeft.blocked() or self.longFrontRight.blocked() == False):
                    self.drift = 0.0


            velo.angular.z = self.drift

        else:


            self.xSpeed = 0

            # Cas où les deux front sont bloqués
            if self.frontLeft.blocked() and self.frontRight.blocked():
                
                if self.frontExtremeLeft.blocked() != self.frontExtremeRight.blocked():
                    
                    self.rotateCount = 5

                    if self.frontExtremeLeft.blocked():
                        # rotate droite
                        self.rotateDir = -0.9
                    else:
                        # rotate gauche
                        self.rotateDir = 0.9
                else:
                
                    # demi tour vers gauche

                    if self.frontLeft.closestDistance > self.frontRight.closestDistance:
                        self.rotateDir = 0.9
                    else:
                        # vers droite
                        self.rotateDir = -0.9
                    
                    self.rotateCount = 10


                

                self.rotateCount -= 1

                print("Rotate mode")

                velo.linear.x = 0.01
                velo.angular.z = self.rotateDir
                self.velocity_publisher.publish(velo)

                s = Led()
                s.value = 3
                self.led1_publisher.publish(s)

                s = Led()
                s.value = 3
                self.led2_publisher.publish(s)
                return


            if self.frontLeft.blocked():

                velo.linear.x = 0.01
                self.rotateCount = 6

                if self.frontExtremeLeft.blocked() == False:
                    self.rotateDir = 0.9
                    print("[MOuvement] GB Rotation vers Gauche")
                else:
                    self.rotateDir = -0.9
                    print("[MOuvement] GB Rotation vers Droite")
            
                velo.linear.x = 0.01
                velo.angular.z = self.rotateDir

                

            if self.frontRight.blocked():
                velo.linear.x = 0.01
                self.rotateCount = 6

                if self.frontExtremeRight.blocked() == False:
                    self.rotateDir = -0.9
                    print("[MOuvement] DB Rotation vers Droite")
                else:
                    self.rotateDir = 0.9
                    print("[MOuvement] DB Rotation vers Gauche")

                velo.linear.x = 0.01
                velo.angular.z = self.rotateDir
                
        self.iterations = self.iterations + 1
        
        
        if self.isFinish():
            self.timer.cancel()
            velo.linear.x = 0.0
            velo.angular.x = 0.0
            self.velocity_publisher.publish(velo)

            self.finish()
            
        else:
            self.velocity_publisher.publish(velo)
        

        
     

def main(args=None):
    rclpy.init(args=args)
    move = MoveNode()



    # Start the ros infinit loop with the move node.
    rclpy.spin_until_future_complete(move, future=move.future)

    # At the end, destroy the node explicitly.
    move.destroy_node()

    # and shut the light down.
    rclpy.shutdown()

if __name__ == '__main__':
    main()
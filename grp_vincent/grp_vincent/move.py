from asyncio import Future
from math import sqrt
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from sensor_msgs.msg import PointCloud


import time
import sys
sys.path.append("/install/kobuki_ros_interfaces/lib/python3.8/site-packages")
from kobuki_ros_interfaces.msg import Sound
from kobuki_ros_interfaces.msg import Led
from kobuki_ros_interfaces.msg import WheelDropEvent

import time
import random


class FuturNode(Node):

    def __init__(self, name):
        super().__init__(name)

        self.future = Future()

    def finish(self):
        self.future.set_result('ok')

'''
Zone de détéction
'''
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

    '''
    Mise à jour
    '''
    def update(self):
        self.count = self.countCopy
        self.closestDistance = self.closestDistanceCopy

        self.countCopy = 0
        self.closestDistanceCopy = 10000.0

    '''
    Il y a un objet dans la zone
    '''
    def blocked(self):
        return self.closestDistance < 10000.0

    '''
    Indique si un point se trouve dans la zone
    '''
    def isInside(self, point):
        return point.x >= self.minx and point.y >= self.miny and point.x <= self.maxx and point.y <= self.maxy

    '''
    Debug
    '''
    def debug(self):
        print(f"State : " + self.name + " => C = " + str(self.count) + " CLOSEST = " + str(self.closestDistance) + ", BLOCK = " + str(self.blocked()) + "")

    '''
    Traitement d'un point
    '''
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

        # Publisher
        self.velocity_publisher = self.create_publisher(Twist, '/multi/cmd_nav', 10)
        self.sound_publisher = self.create_publisher(Sound, '/commands/sound', 10)
        self.led1_publisher = self.create_publisher(Led, '/commands/led1', 10)
        self.led2_publisher = self.create_publisher(Led, '/commands/led2', 10)

        # Reciver
        self.create_subscription(PointCloud, '/laser/pointcloud', self.scan_callback, 10)
        self.create_subscription(WheelDropEvent, '/events/wheel_drop', self.wheel_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmdVelCallback, 10)

        # Clock principale
        self.timer = self.create_timer(0.1, self.activate) # 0.1 seconds to target a frequency of 10 hertz

        # Zones de détection
        OBS_DIST = 0.25
        
        self.frontLeft = ObstacleAreaData(name="Front Left", miny=0.0, maxy=0.19, minx=0.05, maxx=OBS_DIST)
        self.frontRight = ObstacleAreaData(name="Front Right", miny=-0.19, maxy=0.0, minx=0.05, maxx=OBS_DIST)

        self.frontExtremeLeft = ObstacleAreaData(name="Front extreme Left", miny=0.19, maxy=0.4, minx=0.05, maxx=OBS_DIST)
        self.frontExtremeRight = ObstacleAreaData(name="Front extreme Right", miny=-0.4, maxy=-0.19, minx=0.05, maxx=OBS_DIST)

        self.frontPath = ObstacleAreaData(name="Front path", miny=-0.16, maxy=0.16 ,minx=0.1, maxx= 0.45)

        self.longObstacle = ObstacleAreaData(name="Long obs", miny=-0.2, maxy=0.2,minx=OBS_DIST, maxx=OBS_DIST + 0.4)
        
        self.longFrontLeft = ObstacleAreaData(name="Long front left obs", miny=0.0, maxy=0.25,minx=OBS_DIST, maxx=OBS_DIST + 0.4)
        self.longFrontRight = ObstacleAreaData(name="Long front right obs", miny=-0.25, maxy=0.0,minx=OBS_DIST, maxx=OBS_DIST + 0.4)
        self.longFrontLeft.projectedDist = True
        self.longFrontRight.projectedDist = True

        # Ordre de mouvement (liés au pathfinder) valeur et date
        self.lastOrder = Twist()
        self.lastOrderTime = time.time() + 5 # +5 permet d'attendre 5s avant de lancer robot au démarrage

        # Temps de blocage du robot (permet lancer demi-tour)
        self.blockTime = 0

        # Contient l'état drop du robot
        self.droped = False

        # Gestion du drift (valeur et nombre d'iterations)
        self.drift = 0.0
        self.driftCount = 0
        
        # Gestion de la rotation (demi-tour) (direction et nombre d'iterations)
        self.rotateDir = 0.0
        self.rotateCount = 0
        self.alternateDir = False

        # Vitesse linéaire x
        self.xSpeed = 0.0
    '''
    Callback cmd_vel (Contrôleur de mouvement)
    '''
    def cmdVelCallback(self, twist):
        print("Nouvel ordre reçu : " + str(twist.linear.x))
        self.lastOrder = twist

        # Augmente un peu la vitesse
        self.lastOrder.linear.x *= 1.5
        self.lastOrder.angular.z *= 1.6

        # Notification sonore
        if time.time() - self.lastOrderTime > 5:
            s = Sound()
            s.value = 5
            self.sound_publisher.publish(s)

        # Stoquer le moment de l'ordre pour repasser en mode auto 5s après
        self.lastOrderTime = time.time()

    '''
    Callback quand les roues sont libérées
    '''
    def wheel_callback(self, data):
        if data.state == 1:
            self.droped = True
        else: self.droped = False


    '''
    Callback du laser
    '''
    def scan_callback(self, pc):

        # Mise à jour des zones
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
        self.longFrontLeft.update()
        self.longFrontRight.update()

        # Mise à jour des leds selon l'état des zones
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



        # Debug
        '''print("--------------------------------------------------------")
        self.frontLeft.debug()
        self.frontRight.debug()
        self.frontExtremeLeft.debug()
        self.frontExtremeRight.debug()
        self.longFrontLeft.debug()
        self.longFrontRight.debug()
        self.frontPath.debug()'''


        # Détection blockage robot
        if self.frontLeft.blocked() == False and self.frontRight.blocked() >= False:
            self.blockTime = 0
        else:
            self.blockTime += 1

    def activate(self):
        
        # S'il est drop on ne fait rien
        if self.droped == True:
            return;

        velo = Twist()

        # Pas d'ordre depuis 5min, on passe en mode découverte
        if time.time() - self.lastOrderTime >= 5:

            # Maj des compteurs

            if self.driftCount > 0:
                self.driftCount -= 1

            if self.rotateCount > 0:
                self.rotateCount -= 1

            # Detection d'une situation de blockage
            if self.blockTime > 5:
                velo.linear.x = 0.01

                # Requête de rotation
                self.rotateCount = 20
                self.blockTime = 0

                if self.rotateDir == 0.0:
                    if random.randint(0, 1) == 0:
                        self.rotateDir = 1.5
                    else:
                        self.rotateDir = -1.5



            # Traitement requête rotation
            if self.rotateCount > 0:
                velo.linear.x = 0.01
                velo.angular.z = self.rotateDir

                # Arrêt demi-tour car chemin libre
                if self.frontPath.blocked() == False:
                    velo.linear.x = 0.2
                    self.rotateCount = 0
                    velo.angular.z = 0.0
                    

                self.velocity_publisher.publish(velo)
                return

            # N'est pas dans une situation de blockage
            if self.frontLeft.blocked() == False and self.frontRight.blocked() == False:   

                # Evolution de la vitesse linéaire x
                if self.xSpeed == 0:
                    self.xSpeed = 0.05

                if self.xSpeed < 0.4 and self.longObstacle.blocked() == False:
                    self.xSpeed += 0.05
                
                if self.longObstacle.blocked() and self.xSpeed > 0.3:
                    self.xSpeed -= 0.05

                if self.frontPath.blocked() and self.xSpeed > 0.1:
                    self.xSpeed -= 0.05

                
            
                # Application vitesse linéaire x
                velo.linear.x = self.xSpeed
                
                # Traitement drift terminé, on regarde si on peut crée un nouveau drift
                if self.driftCount <= 0:

                    # Calcul d'une nouvelle valeur de drift


                    # Obstacle lointain à gauche (donc drifter à droite) si il n'y a rien a proximité droite
                    if ((self.longFrontLeft.count > 0 and self.longFrontLeft.closestDistance < self.longFrontRight.closestDistance) or self.frontExtremeLeft.blocked()) and self.frontExtremeRight.blocked() == False:
                        self.drift = -0.45 * 1 / self.longFrontLeft.closestDistance

                        # plus de force
                        if self.frontExtremeLeft.blocked():
                            self.drift -= 0.1

                        self.driftCount = 5


                    # obstacle lointain à droite (donc on veut drifter à gauche) si il n'y a rien a proximité gauche
                    elif ((self.longFrontRight.count > 0 and self.longFrontRight.closestDistance < self.longFrontLeft.closestDistance) or self.frontExtremeRight.blocked()) and self.frontExtremeLeft.blocked() == False:
                        self.drift = 0.45 * 1 / self.longFrontRight.closestDistance
                    
                        # plus de force
                        if self.frontExtremeRight.blocked():
                            self.drift += 0.1

                        self.driftCount = 5

                    else:
                        self.drift = 0.0
                else:

                    # Traitement requête drift

                    # drift vers droite et droite bloqué OU plus l'obs, on annule
                    if self.drift < 0 and (self.frontExtremeRight.blocked() or self.longFrontLeft.blocked() == False):
                        self.drift = 0.0

                    # drift vers gauche et gauche bloqué Ou plus l'obs, on annule
                    if self.drift > 0 and (self.frontExtremeLeft.blocked() or self.longFrontRight.blocked() == False):
                        self.drift = 0.0

                # Application valeur drift
                velo.angular.z = self.drift

            else:
                # On arrête d'avancer
                self.xSpeed = 0.0

                # Cas où les deux front sont bloqués
                if self.frontLeft.blocked() and self.frontRight.blocked():
                    
                    if self.frontExtremeLeft.blocked() != self.frontExtremeRight.blocked():
                        
                        # Requête de rotation (minimum 5 rotations)
                        self.rotateCount = random.randint(5, 10)

                        if self.frontExtremeLeft.blocked():
                            # rotate droite
                            self.rotateDir = -0.9
                        else:
                            # rotate gauche
                            self.rotateDir = 0.9
                    else:
                        
                        # Requête de rotation (minimum 5 rotations)
                        self.rotateCount = random.randint(5, 10)

                        if self.frontLeft.closestDistance > self.frontRight.closestDistance:
                            # vers gauche
                            self.rotateDir = 0.9
                        else:
                            # vers droite
                            self.rotateDir = -0.9
            
                    self.rotateCount -= 1

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

                # bloqué gauche
                if self.frontLeft.blocked():

                    velo.linear.x = 0.01
                    self.rotateCount = random.randint(6, 15)

                    if self.frontExtremeLeft.blocked() == False:
                        self.rotateDir = 0.9
                    else:
                        self.rotateDir = -0.9
                
                    velo.angular.z = self.rotateDir

                    
                # bloqué droite
                if self.frontRight.blocked():
                    velo.linear.x = 0.01
                    self.rotateCount = random.randint(6, 15)

                    if self.frontExtremeRight.blocked() == False:
                        self.rotateDir = -0.9
                    else:
                        self.rotateDir = 0.9

                    velo.angular.z = self.rotateDir


                
        else:
            # traitement ordre
            velo = self.lastOrder
            
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
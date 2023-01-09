from asyncio import Future
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from sensor_msgs.msg import PointCloud

import time

class FuturNode(Node):

    def __init__(self, name):
        super().__init__(name)

        self.future = Future()

    def finish(self):
        self.future.set_result('ok')


class MoveNode(FuturNode):

    def __init__(self):
        super().__init__('move')
        self.velocity_publisher = self.create_publisher(Twist, '/mobile_base/commands/velocity', 10)


        self.create_subscription(PointCloud, '/laser/pointcloud', self.scan_callback, 10)

        self.iterations = 0
        self.timer = self.create_timer(0.1, self.activate) # 0.1 seconds to target a frequency of 10 hertz
        self.leftBlocked = False
        self.rightBlocked = False
        
        self.alternateDir = False

        self.longRangeObs = False

        #self.turning = False
        self.xSpeed = 0.0

        self.turnoverTime = 0

        self.blockTime = 0

        self.lastAlternateDirChange = 0

    def isFinish(self):
        return self.iterations > 20000

    def scan_callback(self, pc):

        # pc.points
    
        # chercher objet à tracker


        tempLeftBlocked = False
        tempRightBlocked = False

        longRangeObs = False

        for point in pc.points:
            if point.y >= -0.15 and point.y <= 0.0 and point.x > 0.05 and point.x < 0.25:
                tempLeftBlocked = True
                print("left blocked")
            

            if point.y >= 0.0 and point.y <= 0.15 and point.x > 0.05 and point.x < 0.25:
                tempRightBlocked = True
                print("right blocked")


            if abs(point.y) < 0.20 and point.x >= 0.25 and point.x < 0.5:
                longRangeObs = True


        self.leftBlocked = tempLeftBlocked
        self.rightBlocked = tempRightBlocked
        self.longRangeObs = longRangeObs

        if self.leftBlocked == False and self.rightBlocked == False:
            self.blockTime = 0
        else:
            self.blockTime += 1

            print(f"Block time : {self.blockTime}")

    def activate(self):
        
        
        
        velo = Twist()


        if self.leftBlocked == False and self.rightBlocked == False:
            
            print("[Mouvement] Avance")

            if self.xSpeed == 0:
                self.xSpeed = 0.03

            if self.longRangeObs == False:

                if self.xSpeed < 0.6:
                    self.xSpeed += 0.03
            

            else:
                
                if self.xSpeed > 0.3:
                    self.xSpeed -= 0.05

                else:  
                    if self.xSpeed < 0.3:
                        self.xSpeed += 0.05

            velo.linear.x = self.xSpeed # target a 0.2 meter per second velocity


        else:

            self.xSpeed = 0


            if self.rightBlocked == False and self.leftBlocked == True:
                velo.linear.x = 0.01
                velo.linear.y = 0.0
                velo.linear.z = 0.0
                velo.angular.x = 0.0
                velo.angular.y = 0.0
                velo.angular.z = 0.9
                print("[MOuvement] Rotation GAUCHE")
            
            
            if self.rightBlocked == True and self.leftBlocked == False:
                velo.linear.x = 0.01
                velo.linear.y = 0.0
                velo.linear.z = 0.0
                velo.angular.x = 0.0
                velo.angular.y = 0.0
                velo.angular.z = -0.9
                print("[MOuvement] Rotation Droite")

          
            if self.rightBlocked == True and self.leftBlocked == True:
                velo.linear.x = 0.01
                velo.linear.y = 0.0
                velo.linear.z = 0.0
                velo.angular.x = 0.0
                velo.angular.y = 0.0

                if time.time() - self.lastAlternateDirChange > 6:
                    self.lastAlternateDirChange = time.time()
                
                    if self.alternateDir:
                        self.alternateDir = False
                    else:
                        self.alternateDir = True


                if self.alternateDir == True:
                    velo.angular.z = 0.9
                else:
                    velo.angular.z = -0.9
                    
                    
                print("[MOuvement] Rotation face")
                

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
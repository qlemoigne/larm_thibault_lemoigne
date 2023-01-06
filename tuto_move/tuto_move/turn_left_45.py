from asyncio import Future
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class FuturNode(Node):

    def __init__(self, name):
        super().__init__(name)

        self.future = Future()

    def finish(self):
        self.future.set_result('ok')

class TurnLeft45Node(FuturNode):

    def __init__(self):
        super().__init__('move')
        self.velocity_publisher = self.create_publisher(Twist, '/mobile_base/commands/velocity', 10)

        self.iterations = 0
        self.timer = self.create_timer(0.1, self.activate) # 0.1 seconds to target a frequency of 10 hertz
        

    def activate(self):
        
        if self.iterations > 10:
            self.timer.cancel()
            self.finish()

        self.iterations += 1;

        velo = Twist()
        velo.linear.x = 0.01
        velo.linear.y = 0.0
        velo.linear.z = 0.0
        velo.angular.x = 0.0
        velo.angular.y = 0.0
        velo.angular.z = 0.785

        self.velocity_publisher.publish(velo)

        

        

        print("Envoi packet turn left 45")
     

def main(args=None):
    rclpy.init(args=args)
    move = TurnLeft45Node()

    

    # Start the ros infinit loop with the move node.
    rclpy.spin_until_future_complete(move, future=move.future)

    # At the end, destroy the node explicitly.
    move.destroy_node()

    # and shut the light down.
    rclpy.shutdown()

if __name__ == '__main__':
    main()
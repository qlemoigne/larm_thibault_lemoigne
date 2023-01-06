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


class MoveNode(FuturNode):

    def __init__(self):
        super().__init__('move')
        self.velocity_publisher = self.create_publisher(Twist, '/mobile_base/commands/velocity', 10)

        self.iterations = 0
        self.timer = self.create_timer(0.1, self.activate) # 0.1 seconds to target a frequency of 10 hertz

    def isFinish(self):
        return self.iterations > 5

    def activate(self):
        velo = Twist()
        velo.linear.x = -0.1 # target a 0.1 meter per second velocity

        self.iterations = self.iterations + 1
        
        
        if self.isFinish():
            self.timer.cancel()
            velo.linear.x = 0.0
            self.velocity_publisher.publish(velo)
            # rclpy.shutdown()

            self.finish()
            
        else:
            self.velocity_publisher.publish(velo)
        

        print("Envoi packet velo")
     

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
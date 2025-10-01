import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class P3ATController(Node):
    def __init__(self):
        super().__init__('p3at_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 1.0  # segundos
        self.timer = self.create_timer(timer_period, self.move)

    def move(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        self.publisher_.publish(msg)
        self.get_logger().info('Movendo P3AT: frente e girando')

def main(args=None):
    rclpy.init(args=args)
    node = P3ATController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


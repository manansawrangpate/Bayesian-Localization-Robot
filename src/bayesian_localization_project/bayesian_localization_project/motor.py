import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_cmd)
        self.get_logger().info('Motor node started')

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = 0.08
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
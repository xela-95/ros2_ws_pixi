import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class DrawCircleNode(Node):

    def __init__(self):
        super().__init__("draw_circle")
        self._cmd_vel_pub = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self._timer = self.create_timer(0.5, self.send_velocity_cmd)
        self.get_logger().info("Draw Circle Node Started")

    def send_velocity_cmd(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        self._cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DrawCircleNode()
    rclpy.spin(node)
    rclpy.shutdown()

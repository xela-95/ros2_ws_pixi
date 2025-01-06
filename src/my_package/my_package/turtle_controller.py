from functools import partial

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import SetPen


class TurtleControllerNode(Node):

    def __init__(self):
        super().__init__("turtle_controller")
        self._prev_x = 0.0
        self._pose_subscriber = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10
        )
        self._cmd_vel_pub = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.get_logger().info("Turtle Controller Node Started")

    def pose_callback(self, pose: Pose):
        cmd = Twist()
        if pose.x > 9.0 or pose.x < 2.0 or pose.y > 9.0 or pose.y < 2.0:
            cmd.linear.x = 1.0
            cmd.angular.z = 0.9
        else:
            cmd.linear.x = 5.0
            cmd.angular.z = 0.0
        self._cmd_vel_pub.publish(cmd)

        if pose.x > 5.5 and self._prev_x <= 5.5:
            self._prev_x = pose.x
            self.get_logger().info("Set pen to red")
            self.call_set_pen_service(255, 0, 0, 3, 0)
        elif pose.x <= 5.5 and self._prev_x > 5.5:
            self._prev_x = pose.x
            self.get_logger().info("Set pen to green")
            self.call_set_pen_service(0, 255, 0, 3, 0)

    def call_set_pen_service(self, r, g, b, width, off):
        client = self.create_client(SetPen, "turtle1/set_pen")
        while not client.service_is_ready():
            self.get_logger().warn("Waiting for service...")
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_set_pen))

    def callback_set_pen(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

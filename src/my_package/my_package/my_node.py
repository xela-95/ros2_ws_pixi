import rclpy
from rclpy.node import Node


class MyNode(Node):
    def __init__(self):
        super().__init__("my_node")
        self._counter = 0
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Hello " + str(self._counter))
        self._counter += 1


def main(args=None):
    # Initialize ROS communication
    rclpy.init(args=args)
    # Create a node
    node = MyNode()
    # Spin
    rclpy.spin(node)
    # Shutdown ROS communication
    rclpy.shutdown()


if __name__ == "__main__":
    main()

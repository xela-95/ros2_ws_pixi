import rclpy
from rclpy.node import Node


class MyNode(Node):
    def __init__(self):
        super().__init__("my_node")
        self.get_logger().info("ROS2")


def main(args=None):
    # Initialize ROS communication
    rclpy.init(args=args)

    # Create a node
    node = MyNode()

    rclpy.spin(node)
    # Shutdown ROS communication
    rclpy.shutdown()


if __name__ == "__main__":
    main()

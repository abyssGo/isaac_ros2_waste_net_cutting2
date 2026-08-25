import rclpy
from rclpy.node import Node


class Vision2Node(Node):

    def __init__(self):
        super().__init__('vision2_node')

        self.get_logger().info('NetClean vision2_node started')


def main(args=None):
    rclpy.init(args=args)

    node = Vision2Node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

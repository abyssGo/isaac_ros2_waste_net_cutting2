import rclpy
from rclpy.node import Node


class Robot1Node(Node):

    def __init__(self):
        super().__init__('robot1_node')

        self.get_logger().info('NetClean robot1_node started')


def main(args=None):
    rclpy.init(args=args)

    node = Robot1Node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

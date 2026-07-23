import rclpy as ros2
from rclpy.node import Node


class SLAMNode(Node):

    def __init__(self):
        super().__init__('slam')


def main(args=None):
    ros2.init(args=args)

    node = SLAMNode()
    ros2.spin(node)

    node.destroy_node()
    ros2.shutdown()


if __name__ == '__main__':
    main()
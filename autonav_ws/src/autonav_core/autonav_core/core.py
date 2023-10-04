from rclpy.node import Node


class AutonavNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

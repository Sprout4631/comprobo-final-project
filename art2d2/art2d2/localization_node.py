import rclpy
from rclpy.node import Node

class LocalizationNode(Node):   
    def __init__(self):
        super().__init__('localization_node')

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
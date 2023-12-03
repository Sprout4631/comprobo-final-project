import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped, PoseStamped, Pose, Point

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class LocalizationNode(Node):   
    def __init__(self):
        super().__init__('localization_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_publisher = self.create_publisher(PoseStamped, "/pose_estimate", 10)

        self.timer = self.create_timer(0.05, self.on_timer)

    def on_timer(self):

        to_frame = "map"
        from_frame = "base_footprint"

        try:
            t = self.tf_buffer.lookup_transform(
                to_frame,
                from_frame,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame} to {from_frame}: {ex}')
            return

        # print(t)

        translation = t.transform.translation
        rotation = t.transform.rotation
        posn_point = Point(x=translation.x, y=translation.y, z=translation.z)        
        pose = Pose(position=posn_point, orientation=rotation)
        pose_msg_out = PoseStamped(header=t.header, pose=pose)

        self.pose_publisher.publish(pose_msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
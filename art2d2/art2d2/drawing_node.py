import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import Pose
from helper import euler_from_quaternion

class DrawingNode(Node):

    current_pose: Pose = Pose()
    current_angle = 0.
    target_angle = 0.
    current_point = 0,
    target_point = 0.
    waypt_index = 0

    waypoints = np.array([
        [0.5, 0.],
        [0., 0.5],
        [0., 0.]
    ])
    
    def __init__(self):
        super().__init__('drawing_node')

        self.pose_subscriber = self.create_subscription(Pose, '/pose', self.pose_callback, 10)
            # TODO: FIgure out the topic and message type
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.move)


    def pose_callback(self, pose_msg):
        """
        Store the latest pose published by the localization node
        
        Args:
            self: node object
            pose_msg: Pose
        """
        self.current_pose = pose_msg

    def rotate_to_angle(self, target_angle):
        """
        Rotates from current angle to desired angle using proportional control
        
        Args:
            self: node object
            target_angle: pose object that represents desired angle
        
        Returns:
            True if current angle is within threshold of desired angle
        """
        pass

    def drive_to_point(self, target_point):
        """
        Drives in a straight line from current point to desired point using
        proportional control

        Args:
            self: node object
            target_point: pose object that represents desired point
        
        Returns:
            True if current location is within threshold of desired point
        """
        pass

    def move(self):
        """
        Moves robot to next waypoint by calling rotation and driving functions,
        loops on every timer tick
        """
        self.rotate_to_angle(self.target_angle)
        self.drive_to_point(self.target_point)
        pass
    
    def increment_waypoint(self):
        """
        Increments which waypoint the robot is moving towards.
        """
        self.current_point = np.array([self.current_pose.x, self.current_pose.y])
        current_roll, current_pitch, current_yaw = euler_from_quaternion(self.current_pose)
        self.current_angle = current_yaw

        self.target_point = self.waypoints[self.waypt_index, :] #takes the whole row at this index
        delta = self.target_point-self.current_point
        self.target_angle = np.arctan2(delta[1], delta[0])

        self.waypt_index += 1

def main(args=None):
    rclpy.init(args=args)
    node = DrawingNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
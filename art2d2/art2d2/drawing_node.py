import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import Pose, Twist
from art2d2.helper import euler_from_quaternion

class DrawingNode(Node):

    current_pose: Pose = Pose()
    current_angle = 0.
    target_angle = 0.
    current_point = np.array([0,0])
    target_point = np.array([0,0])
    waypt_index = 0
    angle_error = 0

    state = "driving"

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

        # Publish to cmd_vel topic
        self.vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)


    def pose_callback(self, pose_msg):
        """
        Store the latest pose published by the localization node
        
        Args:
            self: node object
            pose_msg: Pose
        """
        self.current_pose = pose_msg

        current_roll, current_pitch, current_yaw = euler_from_quaternion(self.current_pose)
        self.current_angle = current_yaw

    def rotate_to_angle(self):
        """
        Rotates from current angle to desired angle using proportional control
        
        Args:
            self: node object
            target_angle: pose object that represents desired angle
        
        Returns:
            angle_error: difference between desired angle and current angle
        """
        # 1. Find angle error
        # 2. Find how fast we should turn
        # 3. Publish the new velocity command
        # 4. return the angle error
        angle_error = self.target_angle - self.current_angle
        k_p = 0.1
        vel_out = Twist()
        vel_out.angular.z = k_p * angle_error
        self.vel_publisher.publish(vel_out)
        return angle_error

    def drive_to_point(self):
        """
        Drives in a straight line from current point to desired point using
        proportional control

        Args:
            self: node object
            target_point: pose object that represents desired point
        
        Returns:
            position_error: difference between desired position and current
                position
        """
        position_error = np.linalg.norm(self.target_point - self.current_point)
        k_p = 0.5
        vel_out = Twist()
        vel_out.linear.x = k_p * position_error
        self.vel_publisher.publish(vel_out)
        return position_error

    def move(self):
        """
        Moves robot to next waypoint by calling rotation and driving functions,
        loops on every timer tick
        """
        if self.state == "turning":
            angle_error = self.rotate_to_angle()
            
            if angle_error < 0.1:
                self.state == "driving"
                
        elif self.state=="driving":
            position_error = self.drive_to_point()
            
            if position_error < 0.03:
                if self.waypt_index < len(self.waypoints):
                    # Get next waypoint and turn
                    self.increment_waypoint()
                    self.state == "turning"
                else:
                    # Set velocities to zero
                    # Kill the node
                    zero_vel = Twist()
                    self.vel_publisher.publish(zero_vel)
                    self.destroy_node()
    
    def increment_waypoint(self):
        """
        Increments which waypoint the robot is moving towards.
        """
        self.current_point = np.array([self.current_pose.position.x, self.current_pose.position.y])

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
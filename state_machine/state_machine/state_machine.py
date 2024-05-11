import rclpy
import numpy as np
from rclpy.node import Node

from geometry_msgs.msg import PoseArray
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped

STOPPED = 0
SAFETY_STOPPED = 1
DRIVING = 2

class StateMachine(Node):   
    
    stop_array = [False, False]
     
    def __init__(self):
        super().__init__('state_machine')
        
        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", 10) # TODO: change to car drive

        self.goal_points_sub = self.create_subscription(PoseArray, "/shell_points", self.goal_points_cb, 1)

        self.stop_detector_sub = self.create_subscription(Bool, "/stop_detection", self.stop_detector_cb, 1)
        self.safety_stop_sub = self.create_subscription(Bool, "/safety_stop", self.safety_stop_cb, 1)

        self.follower_sub = self.create_subscription(AckermannDriveStamped, "/follower_drive", self.follower_cb, 1)        
        
        self.timer = self.create_timer(0.01, self.timer_cb)
        # self.state = STOPPED
        self.follower_drive_cmd = self.create_drive_msg(0, 0)
        self.goal_points = []

    def timer_cb(self):
        if any(self.stop_array):
        # if self.state == STOPPED:
            self.drive_pub.publish(self.create_drive_msg(0, 0))
        # elif self.state == SAFETY_STOPPED:
        #     start_safety_stop = self.get_clock().now().to_msg().sec
        #     while self.get_clock().now().to_msg().sec - start_safety_stop < 1:
        #         self.drive_pub.publish(self.create_drive_msg(0, 0))
        #     start_backup = self.get_clock().now().to_msg().sec
        #     while self.get_clock().now().to_msg().sec - start_backup < 1:
        #         self.drive_pub.publish(self.create_drive_msg(-1.5, 0))
        # elif self.state == DRIVING:
        else:
            self.drive_pub.publish(self.follower_drive_cmd)

    def stop_detector_cb(self, msg):
        self.stop_array[STOPPED] = msg.data
        # if msg.data and self.state != SAFETY_STOPPED:
            # self.state == STOPPED

    def safety_stop_cb(self, msg):
        self.stop_array[SAFETY_STOPPED] = msg.data
        # if msg.data:
            # self.state == SAFETY_STOPPED

    def goal_points_cb(self, msg):
        self.goal_points = [(pose.position.x, pose.position.y) for pose in msg.poses]

    def follower_cb(self, msg):
        self.follower_drive_cmd = msg
    
    def create_drive_msg(self, vel, angle):
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'map'
        cmd.drive.steering_angle = float(angle)
        cmd.drive.speed = float(vel)
        return cmd

def main(args=None):
    rclpy.init(args=args)

    state_machine = StateMachine()

    rclpy.spin(state_machine)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    state_machine.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
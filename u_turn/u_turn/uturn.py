import rclpy
from rclpy.node import Node
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

IDLE = 0
TURNING = 1
BACKING_UP = 2

class UTurn(Node):
    def __init__(self):
        super().__init__("parking_controller")
        
        self.done_pub = self.create_publisher(String, "/uturn_is_done", 1)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", 1)
        self.create_subscription(Bool, "/safety_stop", self.safety_cb, 1)
        self.create_subscription(String, "/uturn", self.start_cb, 1)
        self.create_timer(0.05, self.loop)

        self.state = IDLE
        self.is_stop = False
        self.start_angle = 0.0

        self.start_backing_up = None
        self.backing_up = False
        self.last_safety_msg = False
        self.start_turning = 0.0

        self.get_logger().info("Started")

    def loop(self):
        self.get_logger().info(f"{self.state}")
        if self.state == IDLE:
            return 
        elif self.state == TURNING:
            if self.get_time() - self.start_turning < 1.5:
                self.drive_pub.publish(self.create_drive_msg(1.5, 100))
            msg = String()
            msg.data = "done"
            self.done_pub.publish(msg)
            self.state = IDLE
        elif self.state == BACKING_UP:
            if self.get_time() - self.start_backing_up < 1.0: 
                self.drive_pub.publish(self.create_drive_msg(-1.5, -.15))
            msg = String()
            msg.data = "done"
            self.done_pub.publish(msg)
            self.state = IDLE

    def safety_cb(self, msg):
        if msg.data and self.last_safety_msg != msg.data:
            self.state = BACKING_UP
            self.start_backing_up = self.get_time()
        self.last_safety_msg = msg.data

    def start_cb(self, msg):
        if msg.data == "start":
            self.start_turning = self.get_time()
            self.state = TURNING

    def create_drive_msg(self, vel, angle):
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'map'
        cmd.drive.steering_angle = float(angle)
        cmd.drive.speed = float(vel)
        return cmd
    
    def get_time(self):
        return self.get_clock().now().to_msg().sec + (self.get_clock().now().to_msg().nanosec * (10**-9))

def main(args=None):
    rclpy.init(args=args)
    node = UTurn()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
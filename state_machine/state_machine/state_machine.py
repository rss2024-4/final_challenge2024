import rclpy
import math
import numpy as np
from rclpy.node import Node

from std_msgs.msg import Float32
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped


class StateMachine(Node):
    
    states = set()
    stops = [False, False, False]
    
    def __init__(self):
        super().__init__('state_machine')
        
        self.stop_sub = self.create_subscription(Bool, 'safety_stop_bool', self.safety_stop_callback, 10) # Publish stop topic
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10) # TODO: change to car drive
        self.uturnpub = self.create_publisher(Bool, 'uturn_bool', 10) # Publish U-turn
        self.uturnsub = self.create_subscription(Bool, 'uturn_bool', self.uturn_callback, 10) # Publish stop topic
        
        self.timer = self.create_timer(0.01, self.run)
        
    def run(self):
        self.get_logger().info(str(self.states) + " | " + str(self.stops))
        
        if any(self.stops):
            self.publishDriveCmd(0, 0) # stop
            return
        
        if False: # check for uturn
            msg = Bool()
            msg.data = True
            self.uturnpub.publish(msg)
                    
            
    def safety_stop_callback(self, msg):
        self.stop_callback(msg.data, 0)
        
    def stop_callback(self, stop, ind):
        self.stops[ind] = stop
            
    def uturn_callback(self, msg):
        if not msg.data:
            self.states.remove('uturn')
        
    def publishDriveCmd(self, vel, angle):
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'map'
        cmd.drive.steering_angle = float(angle)
        cmd.drive.speed = float(vel)
        self.drive_pub.publish(cmd)
        
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
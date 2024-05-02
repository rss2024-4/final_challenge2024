import rclpy
import math
import numpy as np
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class StopController(Node):

    def __init__(self):
        super().__init__('stop_controller')
        self.declare_parameter("scan_topic", "default")
        self.sim_test = True

        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.subscriber = self.create_subscription(LaserScan, self.SCAN_TOPIC, self.scan_callback, 10)
        self.drive_output_sub = self.create_subscription(AckermannDriveStamped,
                                                         "/vesc/high_level/ackermann_cmd_mux/output",
                                                         self.handle_drive_msg,
                                                         10
                                                         )
        self.publisher = self.create_publisher(AckermannDriveStamped, '/vesc/low_level/input/safety', 10)

        if self.sim_test:
            self.subscriber = self.create_subscription(LaserScan,'/scan', self.scan_callback, 10)
            self.publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # self.stop_pub = self.create_publisher()

        self.L = .4
        self.W = .3

        # half width of the stopping arc
        self.delta = (self.W + 0.1)/2
        self.cur_angle = 0
        self.cur_velocity = 0.0
        self.decel = 3
        
        self.theta_low = -math.pi / 4
        self.theta_hi = math.pi / 4
        self.delta_r = 0.25
        self.threshold = 5
        
        if self.sim_test:
            self.cur_velocity = 2.0
            self.cur_angle = -0.05
            
            cmd = AckermannDriveStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = 'map'
            cmd.drive.steering_angle = self.cur_angle
            cmd.drive.speed = self.cur_velocity
            self.publisher.publish(cmd)
    
        self.should_stop = False
        # self.publisher_ = self.create_publisher(Float32, 'stop', 10)

    def handle_drive_msg(self, msg):
        self.cur_angle = msg.drive.steering_angle
        self.cur_velocity = msg.drive.speed

    def scan_callback(self, msg):
        
        stopping_d = self.L + self.cur_velocity**2/(2*self.decel)
        
        i_low = math.floor((self.theta_low - msg.angle_min) / msg.angle_increment)
        i_hi = math.ceil((self.theta_hi - msg.angle_min) / msg.angle_increment)

        count = 0        
        
        for i in range(i_low, i_hi + 1):
            angle_p = msg.angle_min + i * msg.angle_increment
            dist_p = msg.ranges[i]
            
            point = (dist_p * math.cos(angle_p) + self.L, dist_p * math.sin(angle_p))
            
            self.get_logger().info(str(angle_p) + " " + str(dist_p))
            
            if self.cur_angle == 0:
                if point[0] <= stopping_d and abs(point[1] < self.delta_r):
                    count += 1
            else:
                R = self.L/np.tan(self.cur_angle)
                center = (0, R * math.copysign(1, self.cur_angle))
                
                point_R = math.sqrt((point[0] - center[0]) ** 2 + (point[1] - center[1]) ** 2)
                point_angle = math.atan2(point[0] - center[0], point[1] - center[1])
                point_arcl = point_angle * R
                
                if R - self.delta_r <= point_R <= R + self.delta_r and stopping_d >= point_arcl >= 0:
                    count += 1
                    
        self.should_stop = (count >= self.threshold)
            
        if self.should_stop:
            cmd = AckermannDriveStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = 'map'
            cmd.drive.steering_angle = 0.0
            cmd.drive.speed = 0.0
            self.publisher.publish(cmd)
            self.get_logger().info('SAFETY CONTROLLER STOP!')
        else:
            cmd = AckermannDriveStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = 'map'
            cmd.drive.steering_angle = self.cur_angle
            cmd.drive.speed = self.cur_velocity
            self.publisher.publish(cmd)
            

def main(args=None):
    rclpy.init(args=args)

    stop_controller = StopController()

    rclpy.spin(stop_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    stop_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

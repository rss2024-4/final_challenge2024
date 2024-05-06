import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
from sensor_msgs.msg import Image
from detector import StopSignDetector
# from sift_sign import StopSignDetector

from ackermann_msgs.msg import AckermannDriveStamped

DRIVING = 0
WAITING = 1
GOING_PAST = 2

class SignDetector(Node):
    def __init__(self):
        super().__init__("stop_detector")
        self.detector = StopSignDetector()

        # calling safety rn, may public seperate topic later
        self.publisher = self.create_publisher(AckermannDriveStamped, '/vesc/low_level/input/safety', 10)
        
        self.subscriber = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.callback, 1)
        self.bridge = CvBridge()

        self.get_logger().info("Stop Detector Initialized")

        # change to tune
        self.THRESHOLD = 100 # pixels^2
        self.WAITTIME = 3 # seconds

        self.timer = self.create_timer(.001, self.timer_cb)
        self.state = DRIVING

    def callback(self, img_msg):
        # Process image with CV Bridge
        image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

        is_sign, box = self.detector.predict(image)

        if self.state == DRIVING:
            cmd = AckermannDriveStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = 'map'
            cmd.drive.steering_angle = .001
            cmd.drive.speed = 2.0
            self.publisher.publish(cmd)

            if is_sign:
                area = (box[2]-box[0])*(box[3]-box[0])
                self.get_logger().info(f'{area=}')
                if area > self.THRESHOLD:
                    self.state = WAITING
                    self.start_time = self.get_time()
            
        elif self.state == WAITING:
            cmd = AckermannDriveStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = 'map'
            cmd.drive.steering_angle = 0.0
            cmd.drive.speed = 0.0
            self.publisher.publish(cmd)
            self.get_logger().info('STOP SIGN STOP!')

            if(self.get_time() > self.start_time + self.WAITTIME):
                self.state = GOING_PAST
        
        elif self.state == GOING_PAST:
            # can also just be until not is_sign
            if(self.get_time() > self.start_time + 2*self.WAITTIME):
                self.state = DRIVING
                
        
    def get_time(self):
        return self.get_clock().now().to_msg().sec + (self.get_clock().now().to_msg().nanosec * (10**-9))

def main(args=None):
    rclpy.init(args=args)
    detector = SignDetector()
    rclpy.spin(detector)
    rclpy.shutdown()

if __name__=="__main__":
    main()
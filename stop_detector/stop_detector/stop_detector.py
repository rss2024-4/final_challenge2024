import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge, CvBridgeError

import torch

import numpy as np
from sensor_msgs.msg import Image
# from detector import StopSignDetector
# from sift_sign import StopSignSIFT

from ackermann_msgs.msg import AckermannDriveStamped

DRIVING = 0
WAITING = 1
GOING_PAST = 2

class SignDetector(Node):
    def __init__(self):
        super().__init__("stop_detector")
        self.detector = StopSignDetector()
        # self.detector = StopSignSIFT()

        # calling safety rn, may public seperate topic later
        self.publisher = self.create_publisher(AckermannDriveStamped, '/vesc/low_level/input/safety', 10)
        
        self.subscriber = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.callback, 1)
        self.bridge = CvBridge()

        self.get_logger().info("Stop Detector Initialized")

        # change to tune
        self.THRESHOLD = 100 # pixels^2
        self.WAITTIME = 15 # seconds?

        # self.timer = self.create_timer(.001, self.timer_cb)
        self.state = DRIVING

        self.start_time = 0
        self.vroom = True

    def callback(self, img_msg):

        try:
            # Process image with CV Bridge
            image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

            is_sign, box = self.detector.predict(image)

            if self.state == DRIVING:
                self.get_logger().info('vroom vroom')

                if is_sign:
                    self.get_logger().info('i see the sign')

                    area = (box[2]-box[0])*(box[3]-box[1])
                    self.get_logger().info(f'{area=}')
                    # if area > self.THRESHOLD:
                    #     self.state = WAITING
                    #     self.start_time = self.get_time()
                
            elif self.state == WAITING:
                self.get_logger().info('STOP SIGN STOP!')

                if(self.get_time() > self.start_time + self.WAITTIME):
                    self.state = GOING_PAST
            
            elif self.state == GOING_PAST:
                # can also just be until not is_sign
                self.get_logger().info('still vroom vroom')
                if(self.get_time() > self.start_time + 2*self.WAITTIME):
                    self.state = DRIVING
        except:
            self.get_logger().info('error')
                
        
    def get_time(self):
        return int(self.get_clock().now().to_msg().sec) # + (self.get_clock().now().to_msg().nanosec * (10**-9))


class StopSignDetector:
  def __init__(self, threshold=0.5):
    # self.model = torch.hub.load('ultralytics/yolov5', 'yolov5n', pretrained=True)
    self.model = torch.hub.load('/root/.yolo', 'custom', source='local', path='/root/.yolo/yolov5n.pt', force_reload=False)
    self.threshold = threshold
    self.results = None

  def predict(self, img):
    """
    Takes in a path or numpy array representing an image
    returns whether or not there is a stop sign, along with the bounding box surrounding it
    """

    if type(img) == str:
      # Path has been passed in
      img_path = img
      img = read_image(img_path)

    results = self.model(img)
    results_df = results.pandas().xyxy[0]
    self.results = results_df

    return is_stop_sign(results_df, threshold=self.threshold), get_bounding_box(results_df, threshold=self.threshold)

  def set_threshold(self, new_thresh):
    self.threshold=new_thresh


# Utilities

# Image
def read_image(path):
    rgb_im = cv2.cvtColor(cv2.imread(str(path)), cv2.COLOR_BGR2RGB)
    return rgb_im

# Detecting Utils

THRESHOLD = 0.7

def is_stop_sign(df, label='stop sign', threshold=THRESHOLD):
    confidences = df[df['confidence'] > threshold]
    return len(confidences[confidences['name'] == label]) != 0 # If a stop sign has been detected

def get_bounding_box(df, label='stop sign', threshold=THRESHOLD):
    if not is_stop_sign(df, label=label, threshold=threshold): return (0, 0, 0, 0)
    confidences = df[df['confidence'] > threshold]
    stop_sign = confidences[confidences['name'] == label].head(1)
    coords = stop_sign.xmin, stop_sign.ymin, stop_sign.xmax, stop_sign.ymax
    return [coord.values[0] for coord in coords]

def main(args=None):
    rclpy.init(args=args)
    detector = SignDetector()
    rclpy.spin(detector)
    rclpy.shutdown()

if __name__=="__main__":
    main()
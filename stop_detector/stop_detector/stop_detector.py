import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge, CvBridgeError

import torch

import numpy as np
from sensor_msgs.msg import Image
# from detector import StopSignDetector
# from sift_sign import StopSignSIFT

# from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool

DRIVING = 0
WAITING = 1
GOING_PAST = 2
TRAFFIC_STOP = 3

class SignDetector(Node):
    def __init__(self):
        super().__init__("stop_detector")
        self.detector = StopSignDetector()
        # self.detector = StopSignSIFT()

        # calling safety rn, may publish seperate topic later
        # self.publisher = self.create_publisher(AckermannDriveStamped, '/vesc/low_level/input/safety', 10)
        
        self.subscriber = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.callback, 5)
        self.stop_pub = self.create_publisher(Bool, '/stop_detection', 10) # true to stop
        self.timer = self.create_timer(.001, self.timer_cb)
        self.bridge = CvBridge()

        self.get_logger().info("Sign & Light Detector Initialized")

        # change to tune
        self.sign_threshold = 100 # pixels^2
        self.light_threshold = 100 # pixels^2
        self.WAITTIME = 5 # seconds?

        self.state = DRIVING

        self.start_time = 0
    
    def publish_bool(self, shouldStop):
        cmd = Bool()
        cmd.data = shouldStop
        self.stop_pub.publish(cmd)

    def timer_cb(self):
        self.get_logger().info(f'{self.state=}')

    def callback(self, img_msg):
        # Process image with CV Bridge
        image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

        is_sign, sign_box, is_light, light_box = self.detector.predict(image)

        if self.state == DRIVING:
            self.get_logger().info('vroom vroom')
            self.publish_bool(False)

            if is_sign:
                self.get_logger().info('SIGN SIGN SIGN')
                area = (sign_box[2]-sign_box[0])*(sign_box[3]-sign_box[1])
                self.get_logger().info(f'{area=}')
                if area > self.sign_threshold:
                    self.state = WAITING
                    self.start_time = self.get_time()
                    self.publish_bool(True)
            
            elif is_light:
                self.get_logger().info('LIGHT LIGHT LIGHT')
                area = (light_box[2]-light_box[0])*(light_box[3]-light_box[1])
                self.get_logger().info(f'{area=}')
                if area > self.light_threshold:
                    if check_red(image, is_light, light_box, self):
                        self.state = TRAFFIC_STOP
            
        elif self.state == WAITING:
            self.publish_bool(True)
            self.get_logger().info('WAIT WAIT WAITING')

            if(self.get_time() > self.start_time + self.WAITTIME):
                self.state = GOING_PAST
                self.start_time = self.get_time()
        
        # drive past sign/light, ignoring everything and hoping they aren't back to back
        elif self.state == GOING_PAST:
            self.publish_bool(False)
            # may can just be until not is_sign and not is_light?
            self.get_logger().info('PAST PAST PAST')
            if(self.get_time() > self.start_time + self.WAITTIME):
                self.state = DRIVING

        elif self.state == TRAFFIC_STOP:
            self.publish_bool(True)
            self.get_logger().info('RED LIGHT RED LIGHT')
            if not check_red(image, is_light, light_box, self):
                self.state = DRIVING # maybe should be waiting

        else:
            self.get_logger().info('why are you in the else')
                
    def get_time(self):
        return self.get_clock().now().to_msg().sec # + (self.get_clock().now().to_msg().nanosec * (10**-9))


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
    is_sign = is_stop_sign(results_df, "stop sign", self.threshold)
    sign_bb = get_bounding_box(results_df, "stop sign", self.threshold)
    is_light = is_stop_sign(results_df, "traffic light", self.threshold)
    light_bb = get_bounding_box(results_df, "traffic light", self.threshold)


    return is_sign, sign_bb, is_light, light_bb

  def set_threshold(self, new_thresh):
    self.threshold=new_thresh


# Utilities

# Image
def read_image(path):
    rgb_im = cv2.cvtColor(cv2.imread(str(path)), cv2.COLOR_BGR2RGB)
    return rgb_im

# Checking if traffic light is red

def crop_to_bounding(img, bounding_box):
    minx, miny, maxx, maxy = [int(x) for x in bounding_box]
    return img[miny:maxy, minx:maxx]

# splits image in half horizontally
def split_img(img):
    height = img.shape[0]

    height_cutoff = height // 2
    s1 = img[:height_cutoff,:]
    s2 = img[height_cutoff:,:]

    return s1, s2

# returns the ratio of red in the img
def red_percentage(img, node):
    red = [255, 0, 0]
    diff = 15

    # 'shades' of red to find; loaded in BGR
    boundaries = [([red[2], red[1], red[0]-diff],
           [red[2]+diff, red[1]+diff, red[0]])]
    
    for (lower, upper) in boundaries:
        lower = np.array(lower, dtype=np.uint8)
        upper = np.array(upper, dtype=np.uint8)

        mask = cv2.inRange(img, lower, upper)

        ratio_red = cv2.countNonZero(mask)/(img.size/3)
        redness = np.round(ratio_red, 2)
        node.get_logger().info(f'{redness=}')
        return redness

def check_red(img, is_light, light_box, node):
    redness_threshold = .01
    if is_light:
        cropped = crop_to_bounding(img, light_box)
        top = split_img(cropped)[0]
        red_perc = red_percentage(top, node)
        return red_perc >= redness_threshold
    return False

# Detecting Utils

THRESHOLD = 0.7

def is_stop_sign(df, label, threshold=THRESHOLD):
    confidences = df[df['confidence'] > threshold]
    return len(confidences[confidences['name'] == label]) != 0 # If a stop sign has been detected

def get_bounding_box(df, label, threshold=THRESHOLD):
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
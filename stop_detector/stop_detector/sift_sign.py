import os
import cv2
import torch

import numpy as np
import matplotlib.pyplot as plt

from PIL import Image, ImageDraw

class StopSignSIFT:

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

    # results = self.model(img)
    # results_df = results.pandas().xyxy[0]
    # self.results = results_df
    
    template = read_image('sign.jpg')

    return cd_sift_ransac(img, template)

  # def draw_box(self, img, box=None):
  #   if box is None: _, box = self.predict(img)
  #   box_img = draw_box(img, box)
  #   return box_img

  # def set_threshold(self, new_thresh):
  #   self.threshold=new_thresh


def cd_sift_ransac(img, template):
	"""
	Implement the cone detection using SIFT + RANSAC algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the bottom left of the bbox and (x2, y2) is the top right of the bbox
	"""
	# Minimum number of matching features
	MIN_MATCH = 10 # Adjust this value as needed
	# Create SIFT
	sift = cv2.xfeatures2d.SIFT_create()

	# Compute SIFT on template and test image
	kp1, des1 = sift.detectAndCompute(template,None)
	kp2, des2 = sift.detectAndCompute(img,None)

	# Find matches
	bf = cv2.BFMatcher()
	matches = bf.knnMatch(des1,des2,k=2)

	# Find and store good matches
	good = []
	for m,n in matches:
		if m.distance < 0.75*n.distance:
			good.append(m)

	# If enough good matches, find bounding box
	if len(good) > MIN_MATCH:
		src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
		dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

		# Create mask
		M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
		matchesMask = mask.ravel().tolist()

		h, w = template.shape
		pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)

		########## YOUR CODE STARTS HERE ##########

		# using mask from ransac data to do lmed
		M, mask = cv2.findHomography(src_pts, dst_pts, cv2.LMEDS, 5.0, mask)
		matchesMask = mask.ravel().tolist()

		dst = cv2.perspectiveTransform(pts, M)

		# prints visual
		draw_params = dict(matchColor = (0,255,0), # draw matches in green color
					 	   singlePointColor = None,
						   matchesMask = matchesMask, # draw only inliers
						   flags = 2)
		img3 = cv2.drawMatches(template,kp1,img,kp2,good,None,**draw_params)
		# image_print(img3)

		x_min = y_min = float("inf")
		x_max = y_max = 0

		for [[x,y]] in np.int32(dst):
			if x < x_min: x_min = x
			if x > x_max: x_max = x
			if y < y_min: y_min = y
			if y > y_max: y_max = y

		########### YOUR CODE ENDS HERE ###########

		# Return bounding box
		return [True, [x_min, y_min, x_max, y_max]]
	else:

		# print(f"[SIFT] not enough matches; matches: ", len(good))

		# Return bounding box of area 0 if no match found
		return [False, [0, 0, 0, 0]]


# Utilities

# Image
def read_image(path):
    rgb_im = cv2.cvtColor(cv2.imread(str(path)), cv2.COLOR_BGR2RGB)
    return rgb_im

# def draw_rect(im, xmin, ymin, xmax, ymax):
#     box = xmin, ymin, xmax, ymax
#     img = Image.fromarray(im)
#     imgd = ImageDraw.Draw(img)
#     imgd.rectangle(box, outline='red')
#     return img

# def draw_box(im, box):
#     img = Image.fromarray(im)
#     imgd = ImageDraw.Draw(img)
#     imgd.rectangle(box, outline='red')
#     return img

# Detecting Utils

# THRESHOLD = 0.7

# def is_stop_sign(df, label='stop sign', threshold=THRESHOLD):
#     confidences = df[df['confidence'] > threshold]
#     return len(confidences[confidences['name'] == label]) != 0 # If a stop sign has been detected

# def get_bounding_box(df, label='stop sign', threshold=THRESHOLD):
#     if not is_stop_sign(df, label=label, threshold=threshold): return (0, 0, 0, 0)
#     confidences = df[df['confidence'] > threshold]
#     stop_sign = confidences[confidences['name'] == label].head(1)
#     coords = stop_sign.xmin, stop_sign.ymin, stop_sign.xmax, stop_sign.ymax
#     return [coord.values[0] for coord in coords]

if __name__=="__main__":
    detector = StopSignSIFT()

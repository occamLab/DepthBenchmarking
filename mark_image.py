"""
This file loads a captured frame and associated metadata and marks the feature
points on the image.
"""

import json
import numpy as np
import cv2 as cv
from scipy.linalg import inv

frame = cv.imread("frame.jpg")

with open("framemetadata.json") as my_file:
    data = json.load(my_file)

lidar_depth = np.array(data["depthData"])
pose = np.reshape(data["pose"], (4,4)).T
raw_fp = np.array(data["rawFeaturePoints"])
raw_fp = np.hstack((raw_fp, np.ones((raw_fp.shape[0], 1)))).T
projected_fp = np.array(data["projectedFeaturePoints"])
focal_length = data["intrinsics"][0]
offset_x = data["intrinsics"][6]
offset_y = data["intrinsics"][7]

phone_fp = inv(pose) @ raw_fp
camera_fp = np.array((phone_fp[0], -phone_fp[1], -phone_fp[2]))
print(camera_fp)
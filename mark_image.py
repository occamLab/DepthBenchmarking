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
camera_fp = np.array((phone_fp[0], -phone_fp[1], -phone_fp[2])).T

calc_projected_fp = []
for row in camera_fp:
    pixel_x = row[0] * focal_length / row[2] + offset_x + 0.5
    pixel_y = row[1] * focal_length / row[2] + offset_y + 0.5
    calc_projected_fp.append([pixel_x, pixel_y])

calc_projected_fp = np.array(calc_projected_fp)

for row in projected_fp:
    pixel_x = round(row[0])
    pixel_y = round(row[1])
    frame[pixel_x - 4: pixel_x + 4, pixel_y - 4: pixel_y + 4] = [51, 14, 247]

cv.imwrite("featurepoints.jpg", frame)
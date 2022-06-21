"""
This file loads a captured frame and associated metadata and marks the feature
points on the image.
"""

import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as colors
import cv2 as cv
from scipy.linalg import inv
from utils import read_pfm

frame = cv.imread("input/frame.jpg")

with open("framemetadata.json") as my_file:
    data = json.load(my_file)

lidar_depth = np.array(data["depthData"])
pose = np.reshape(data["pose"], (4,4)).T
raw_fp = np.array(data["rawFeaturePoints"])
raw_fp = np.hstack((raw_fp, np.ones((raw_fp.shape[0], 1)))).T
projected_fp = np.around(data["projectedFeaturePoints"]).astype(int)
focal_length = data["intrinsics"][0]
offset_x = data["intrinsics"][6]
offset_y = data["intrinsics"][7]

phone_fp = inv(pose) @ raw_fp
camera_fp = np.array((phone_fp[0], -phone_fp[1], -phone_fp[2])).T

ar_depths = []
calc_projected_fp = []
for row in camera_fp:
    pixel_x = row[0] * focal_length / row[2] + offset_x + 0.5
    pixel_y = row[1] * focal_length / row[2] + offset_y + 0.5
    if 0 <= round(pixel_x) < frame.shape[0] \
        and 0 <= round(pixel_y) < frame.shape[1]:
        ar_depths.append(row[2])
    calc_projected_fp.append([pixel_x, pixel_y])

calc_projected_fp = np.array(calc_projected_fp)

# change path to acutal if using a different computer
inverse_depth = np.array(read_pfm("/Users/occamlab/Documents/ARPointCloud/output/frame.pfm")[0])
midas_depth = np.reciprocal(inverse_depth.copy())

plt.figure()
plt.pcolor(midas_depth, norm=colors.LogNorm(), cmap="PuRd_r")
plt.colorbar()
plt.title("Visualization of MiDaS Depths")

midas_depths_at_feature_points = []
for row in projected_fp:
    pixel_x = row[0]
    pixel_y = row[1]
    if 0 <= pixel_x < frame.shape[0] and 0 <= pixel_y < frame.shape[1]:
        midas_depths_at_feature_points.append(midas_depth[pixel_x, pixel_y])
    cv.circle(frame, (pixel_x, pixel_y), 5, (51, 14, 247), -1)

cv.imwrite("output/featurepoints.jpg", frame)

plt.figure()
plt.scatter(ar_depths, midas_depths_at_feature_points)
plt.xlabel("AR Depth")
plt.ylabel("MiDaS Depth")

plt.show()

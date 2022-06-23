"""
This file loads a captured frame and associated metadata and marks the feature
points on the image.
"""

import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as colors
import cv2 as cv
import pyvista as pv
from scipy.linalg import inv
from utils import read_pfm

# load captured frame
frame = cv.imread("input/frame.jpg")

# load the JSON file
with open("framemetadata.json") as my_file:
    data = json.load(my_file)

# extract data from the JSON file
lidar_data = np.array(data["depthData"])
pose = np.reshape(data["pose"], (4,4)).T
raw_fp = np.array(data["rawFeaturePoints"])
raw_fp = np.hstack((raw_fp, np.ones((raw_fp.shape[0], 1)))).T
projected_fp = np.around(data["projectedFeaturePoints"]).astype(int)
focal_length = data["intrinsics"][0]
offset_x = data["intrinsics"][6]
offset_y = data["intrinsics"][7]

# translate feature points to other coordinate frames
phone_fp = inv(pose) @ raw_fp
camera_fp = np.array((phone_fp[0], -phone_fp[1], -phone_fp[2])).T

# calculate depths and pixels of feature points
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
plt.title("AR Depth vs. MiDaS Depth")
plt.xlabel("AR Depth")
plt.ylabel("MiDaS Depth")

# visualize LiDAR depth data
lidar_depth = []
for row in lidar_data:
    x = row[0]* row[3]
    y = row[1] * row[3]
    z = row[2] * row[3]
    lidar_depth.append([x,y,z])

pv_point_cloud = pv.PolyData(lidar_depth)
pv_point_cloud.plot(render_points_as_spheres=True)

lidar_depth = np.reshape(lidar_depth, (256, 192, 3))[:, :, 2].T * -1

plt.figure()
plt.pcolor(lidar_depth, cmap="PuBu_r")
plt.colorbar()
plt.title("LiDAR Depth")

midas_extracted = []
for i in range(lidar_depth.shape[0]):
    for j in range(lidar_depth.shape[1]):
        midas_extracted.append(midas_depth[round(3.75 + 7.5 * i), round(3.75 + 7.5 * j)])

midas_extracted = np.reshape(midas_extracted, (256, 192))

plt.figure()
plt.scatter(lidar_depth, midas_extracted, label="LiDAR")
plt.scatter(ar_depths, midas_depths_at_feature_points, c="r", label="Feature Points")
plt.title("iPhone Depth vs. MiDaS Depth")
plt.legend()
plt.xlabel("iPhone Depth")
plt.ylabel("MiDaS Depth")

plt.show()

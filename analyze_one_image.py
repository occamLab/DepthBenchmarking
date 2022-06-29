"""
This file performs extensive analysis on a captured frame and its corresponding
MiDaS data. Output includes a 3D visualization of LiDAR data, visualizations
of MiDaS and LiDAR depth maps, a scatter plot of correlations between iPhone
depth and MiDaS depth data, and a map of LiDAR confidence levels of the given
frame. The frame must be run through the MiDaS neural network before running
this file for analysis.
"""

import json
import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv
import pyvista as pv
from scipy.linalg import inv
from scipy.stats import spearmanr
from utils import read_pfm

# load captured frame
frame = cv.imread("input/frame.jpg")

# load the JSON file
with open("framemetadata.json") as my_file:
    data = json.load(my_file)

# extract data from the JSON file
lidar_data = np.array(data["depthData"])
lidar_confidence = np.reshape(data["confData"], (192, 256))
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
ar_depths = np.array(ar_depths)

# change path to acutal if using a different computer
# get the inverse depth from the PFM file
inverse_depth = np.array(read_pfm(\
    "/Users/occamlab/Documents/ARPointCloud/output/frame.pfm")[0])
# replace negative and close-to-zero erroneous values with NaN
inverse_depth[inverse_depth<1] = np.nan
# get the MiDaS depth by taking the reciprocal of the inverse depth
midas_depth = np.reciprocal(inverse_depth)

# create a figure representing the MiDaS depths
plt.figure()
plt.pcolor(midas_depth, cmap="PuRd_r")
plt.colorbar()
plt.title("Visualization of MiDaS Depths")

# scale LiDAR data
lidar_depth = []
for row in lidar_data:
    x = row[0]* row[3]
    y = row[1] * row[3]
    z = row[2] * row[3]
    lidar_depth.append([x,y,z])

# visualize a 3D point cloud of the LiDAR depth data
pv_point_cloud = pv.PolyData(lidar_depth)
pv_point_cloud.plot(render_points_as_spheres=True)

# extract depth in meters from LiDAR data
lidar_depth = np.reshape(lidar_depth, (256, 192, 3))[:, :, 2].T * -1

# create a figure representing the LiDAR depths
plt.figure()
plt.pcolor(lidar_depth, cmap="PuBu_r")
plt.colorbar()
plt.title("LiDAR Depth")

# get MiDaS depth values from pixels with feature points
midas_depths_at_feature_points = []
for row in projected_fp:
    pixel_x = row[0]
    pixel_y = row[1]
    if 0 <= pixel_x < frame.shape[0] and 0 <= pixel_y < frame.shape[1]:
        midas_depths_at_feature_points.append(midas_depth[pixel_x, pixel_y])
        
    cv.circle(frame, (pixel_x, pixel_y), 5, (51, 14, 247), -1)

midas_depths_at_feature_points = np.array(midas_depths_at_feature_points)

# draw circles on the input image
cv.imwrite("output/featurepoints.jpg", frame)

# set a maximum MiDaS depth if there are invalid points
if True in np.isnan(midas_depth):
    midas_depth[midas_depth>=max(midas_depths_at_feature_points)] = np.nan

# scale the MiDaS output to the size of the LiDAR depth data
midas_extracted = []
for i in range(lidar_depth.shape[0]):
    for j in range(lidar_depth.shape[1]):
        midas_extracted.append(midas_depth[round(3.75 + 7.5 * i), \
            round(3.75 + 7.5 * j)])
midas_extracted = np.reshape(midas_extracted, (192, 256))

# print out correlations between LiDAR and MiDaS data
print("Correlation:", \
    np.corrcoef(np.ravel(lidar_depth[~np.isnan(midas_extracted)]), \
    np.ravel(midas_extracted[~np.isnan(midas_extracted)]))[0][1])
print("<5 Corr:", \
    np.corrcoef(np.ravel(lidar_depth[(lidar_depth<5) & (~np.isnan(midas_extracted))]), \
    np.ravel(midas_extracted[(lidar_depth<5) & (~np.isnan(midas_extracted))]))[0][1])
print("Mid-high conf corr:", \
    np.corrcoef(np.ravel(lidar_depth[(lidar_confidence>0) & (~np.isnan(midas_extracted))]), \
    np.ravel(midas_extracted[(lidar_confidence>0) & (~np.isnan(midas_extracted))]))[0][1])
print("High conf corr:", \
    np.corrcoef(np.ravel(lidar_depth[(lidar_confidence==2) & (~np.isnan(midas_extracted))]), \
    np.ravel(midas_extracted[(lidar_confidence==2) & (~np.isnan(midas_extracted))]))[0][1])
print(spearmanr(np.ravel(lidar_depth[~np.isnan(midas_extracted)]), \
    np.ravel(midas_extracted[~np.isnan(midas_extracted)])))
print("ARKit-MiDaS Correlation:", \
    np.corrcoef(np.ravel(ar_depths[~np.isnan(midas_depths_at_feature_points)]), \
    np.ravel(midas_depths_at_feature_points[~np.isnan(midas_depths_at_feature_points)]))[0][1])
print(spearmanr(np.ravel(ar_depths[~np.isnan(midas_depths_at_feature_points)]), \
    np.ravel(midas_depths_at_feature_points[~np.isnan(midas_depths_at_feature_points)])))

# create a plot comparing LiDAR vs MiDaS and Feature Points vs MiDaS
plt.figure()
plt.scatter(lidar_depth, midas_extracted, label="LiDAR", s=0.5, alpha=0.5)
plt.scatter(ar_depths, midas_depths_at_feature_points, c="r", label="ARKit FP")
plt.title("iPhone Depth vs. MiDaS Depth")
plt.legend()
plt.xlabel("iPhone Depth")
plt.ylabel("MiDaS Depth")

# create a plot of the LiDAR confidence levels
plt.figure()
plt.pcolor(lidar_confidence, cmap="RdYlGn")
plt.colorbar()
plt.title("LiDAR Confidence")

plt.figure()
high_conf_lidar = lidar_depth.copy()
high_conf_midas = midas_extracted.copy()
high_conf_lidar[lidar_confidence<2] = np.nan
high_conf_midas[lidar_confidence<2] = np.nan

high_conf_corr_map = ((high_conf_lidar - np.nanmean(high_conf_lidar)) / \
    np.nanstd(high_conf_lidar)) * ((high_conf_midas - \
        np.nanmean(high_conf_midas)) / np.nanstd(high_conf_midas)) / \
        (np.sum(lidar_confidence, where=2) / 2)

plt.pcolor(high_conf_corr_map, cmap="RdYlGn")
plt.colorbar()
plt.title("Corrleation between High Confidence LiDAR and MiDaS")

plt.figure()
less_five_lidar = lidar_depth.copy()
less_five_midas = midas_extracted.copy()
less_five_lidar[lidar_depth>5] = np.nan
less_five_midas[lidar_depth>5] = np.nan

less_five_corr_map = ((less_five_lidar - np.nanmean(less_five_lidar)) / \
    np.nanstd(less_five_lidar)) * ((less_five_midas - \
        np.nanmean(less_five_midas)) / np.nanstd(less_five_midas)) / \
        (np.sum(~np.isnan(less_five_lidar)))
plt.pcolor(less_five_corr_map, cmap="RdYlGn")
plt.colorbar()
plt.title("Corrleation between Close Distance LiDAR and MiDaS")

# show the plots
plt.show()

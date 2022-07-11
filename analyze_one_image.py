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
from math import floor
from scipy.linalg import inv
from scipy.stats import spearmanr
from utils import read_pfm
import open3d as o3d

# load captured frame
frame = cv.imread("input/frame.jpg")
frame = cv.rotate(frame, cv.ROTATE_90_COUNTERCLOCKWISE)

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
    pixel_col = row[0] * focal_length / row[2] + offset_x + 0.5
    pixel_row = row[1] * focal_length / row[2] + offset_y + 0.5
    if 0 <= round(pixel_col) < frame.shape[1] \
        and 0 <= round(pixel_row) < frame.shape[0]:
        ar_depths.append(row[2])
    calc_projected_fp.append([pixel_col, pixel_row])
calc_projected_fp = np.array(calc_projected_fp)
ar_depths = np.array(ar_depths)

# change path to acutal if using a different computer
# get the inverse depth from the PFM file
inverse_depth = np.array(read_pfm(\
    "/Users/angrocki/Desktop/SummerResearch/DepthBenchmarking/output/frame.pfm")[0])
inverse_depth = np.rot90(inverse_depth)
# replace negative and close-to-zero erroneous values with NaN
inverse_depth[inverse_depth<1] = np.nan
# get the MiDaS depth by taking the reciprocal of the inverse depth
midas_depth = np.reciprocal(inverse_depth)
"""
# create a figure representing the MiDaS depths
plt.figure()
plt.pcolor(midas_depth, cmap="PuRd_r")
plt.colorbar()
plt.title("Visualization of MiDaS Depths")
"""
# scale LiDAR data
lidar_depth = []
for row in lidar_data:
    x = row[0]* row[3]
    y = row[1] * row[3]
    z = row[2] * row[3]
    lidar_depth.append([x,y,z])
lidar_depth = np.array(lidar_depth)

# Plot Lidar Depth
pcd = o3d.geometry.PointCloud()
point_cloud = np.asarray(np.array(lidar_depth))
pcd.points = o3d.utility.Vector3dVector(point_cloud)
pcd.estimate_normals()
pcd = pcd.normalize_normals()
o3d.visualization.draw_geometries([pcd])
np.savetxt("lidar_depth.csv", lidar_depth, delimiter=",")

# extract depth in meters from LiDAR data
lidar_depth = np.reshape(lidar_depth, (256, 192, 3))[:, :, 2].T * -1
"""
# create a figure representing the LiDAR depths
plt.figure()
plt.pcolor(lidar_depth, cmap="PuBu_r")
plt.colorbar()
plt.title("LiDAR Depth")
"""
# get MiDaS depth values from pixels with feature points
midas_depths_at_feature_points = []
lidar_depths_at_feature_points = []
lidar_confidence_at_feature_points = []
for row in projected_fp:
    pixel_col = row[0]
    pixel_row = row[1]
    if 0 <= pixel_col < frame.shape[1] and 0 <= pixel_row < frame.shape[0]:
        midas_depths_at_feature_points.append(midas_depth[pixel_row, pixel_col])
        lidar_depths_at_feature_points.append(lidar_depth[floor(pixel_row / 7.5), floor(pixel_col / 7.5)])
        lidar_confidence_at_feature_points.append(lidar_confidence[floor(pixel_row / 7.5), floor(pixel_col / 7.5)])
        inverse_color = tuple(int(x) for x in (255 - frame[pixel_row][pixel_col]))
        cv.circle(frame, (pixel_col, pixel_row), 5, (0, 255, 0), -1)
        cv.putText(frame, str(len(midas_depths_at_feature_points)), \
            (pixel_col, pixel_row), cv.FONT_HERSHEY_COMPLEX, 1, inverse_color)

midas_depths_at_feature_points = np.array(midas_depths_at_feature_points)
lidar_depths_at_feature_points = np.array(lidar_depths_at_feature_points)
lidar_confidence_at_feature_points = np.array(lidar_confidence_at_feature_points)

# draw circles on the input image
cv.imwrite("output/featurepoints.jpg", frame)

if True in np.isnan(midas_depth):
            midas_depth[midas_depth>=2*max(midas_depths_at_feature_points)] = np.nan

# scale the MiDaS output to the size of the LiDAR depth data
midas_extracted = []
for i in range(lidar_depth.shape[0]):
    for j in range(lidar_depth.shape[1]):
        midas_extracted.append(midas_depth[round(3.75 + 7.5 * i), \
            round(3.75 + 7.5 * j)])
midas_extracted = np.reshape(midas_extracted, (192, 256))
'''
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
'''
# create a plot comparing LiDAR vs MiDaS and Feature Points vs MiDaS
plt.figure()
plt.scatter(lidar_depth, midas_extracted, label="LiDAR", s=0.5, alpha=0.5)
plt.scatter(ar_depths, midas_depths_at_feature_points, c="r", label="ARKit FP")
plt.title("iPhone Depth vs. MiDaS Depth")
plt.legend()
plt.xlabel("iPhone Depth")
plt.ylabel("MiDaS Depth")
"""
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
"""
plt.figure()
plt.scatter(ar_depths, lidar_depths_at_feature_points)
plt.xlabel("Feature Points")
plt.ylabel("LiDAR")
plt.title("Feature Points vs All LiDAR")
for i in range(ar_depths.size):
    plt.annotate(str(i), (ar_depths[i], lidar_depths_at_feature_points[i]))

plt.figure()
plt.scatter(ar_depths[lidar_confidence_at_feature_points==2], \
    lidar_depths_at_feature_points[lidar_confidence_at_feature_points==2], c="g")
plt.xlabel("Feature Points")
plt.ylabel("High Confidence LiDAR")
plt.title("Feature Points vs High Confidence LiDAR")
for i in range(ar_depths.size):
    if lidar_confidence_at_feature_points[i] == 2:
        plt.annotate(str(i), (ar_depths[i], lidar_depths_at_feature_points[i]))

# calculate line of best fit
valid_midas_at_fp = midas_depths_at_feature_points[~np.isnan(midas_depths_at_feature_points)]
A = np.vstack([valid_midas_at_fp.ravel(), np.ones(valid_midas_at_fp.size)]).T
m, c = np.linalg.lstsq(A, ar_depths[~np.isnan(midas_depths_at_feature_points)].ravel())[0]
print(f"Midas Absolute Depth = {m}*midas_relative_depth + {c} ")

#plot feature points vs midas
plt.figure()
plt.plot(valid_midas_at_fp, ar_depths[~np.isnan(midas_depths_at_feature_points)], \
    'o', label='Original data', markersize=10)
plt.plot(valid_midas_at_fp, m*valid_midas_at_fp + c, 'r', \
    label= f'Fitted line = ar_depth * {m} + {c}')
plt.xlabel("Midas Relative Depth")
plt.ylabel("AR depth")
plt.title("Midas Relative Depth v. AR Depth")
plt.legend()

# plot midas absolute depth
plt.figure()
midas_absolute = midas_depth * m + c
'''
plt.pcolor(midas_absolute, cmap="PuBu_r")
plt.colorbar()
plt.title("Midas Absolute Depth")
'''
'''
# Change of Equations
#Plot midas = m * 1/d + b
midas_depths_at_feature_points = []
for row in projected_fp:
    pixel_col = row[0]
    pixel_row = row[1]
    if 0 <= pixel_col < frame.shape[1] and 0 <= pixel_row < frame.shape[0]:
        midas_depths_at_feature_points.append(inverse_depth[pixel_row, pixel_col])
midas_depths_at_feature_points = np.array(midas_depths_at_feature_points)
valid_midas_at_fp = midas_depths_at_feature_points[~np.isnan(midas_depths_at_feature_points)]
A = np.vstack([valid_midas_at_fp.ravel(), np.ones(len(valid_midas_at_fp))]).T
m, c = np.linalg.lstsq(A, 1/ar_depths.ravel())[0]


#plot midas absolute depth
plt.figure()
midas_absolute = midas_depth * m + c
plt.pcolor(mida
plt.title("Midas Absolute Depth(OTHER EQUATION)")
'''
#Create midas point cloud 
midas_point_cloud = []

for pixel_row in range(midas_absolute.shape[0]):
    for pixel_col in range(midas_absolute.shape[1]):
        x = (pixel_col * midas_absolute[pixel_row][pixel_col] - offset_x - 0.5) / focal_length
        y = (pixel_row * midas_absolute[pixel_row][pixel_col] - offset_y - 0.5) / focal_length
        midas_point_cloud.append((x, -y, -midas_absolute[pixel_row][pixel_col]))
        
pcd = o3d.geometry.PointCloud()
point_cloud = np.asarray(np.array(midas_point_cloud))
pcd.points = o3d.utility.Vector3dVector(point_cloud)
pcd.estimate_normals()
pcd = pcd.normalize_normals()
o3d.visualization.draw_geometries([pcd])
np.savetxt("midas_point_cloud.csv", midas_point_cloud, delimiter=",")

#show the plots
plt.show()

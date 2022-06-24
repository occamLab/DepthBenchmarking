import os
import json
import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv
from scipy.linalg import inv
from scipy.stats import spearmanr
from utils import read_pfm

user = "HccdFYqmqETaJltQbAe19bnyk2e2"
trial = "6CCBDFF7-057E-4FB6-A00F-98E659CE5A88"
frame_number = "0009"
data_path = "/Users/angrocki/Desktop/DepthData/depth_benchmarking"
project_path = "/Users/angrocki/Desktop/SummerResearch/DepthBenchmarking"
midas_input_path = project_path+ "/input/"
midas_output_path = project_path + "/output/"
name = user[0:3] + trial[0:3] + frame_number

# check if graph directory exists
if not os.path.exists(f"{data_path}/{user}/{trial}/data"):
    os.makedirs(f"{data_path}/{user}/{trial}/data")

# load captured frame
frame = cv.imread(midas_input_path + "frame.jpg")

# load the JSON file
with open(f"{data_path}/{user}/{trial}/{frame_number}/framemetadata.json") as my_file:
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
for row in camera_fp:
    pixel_x = row[0] * focal_length / row[2] + offset_x + 0.5
    pixel_y = row[1] * focal_length / row[2] + offset_y + 0.5
    if 0 <= round(pixel_x) < frame.shape[0] \
        and 0 <= round(pixel_y) < frame.shape[1]:
        ar_depths.append(row[2])

# change path to acutal if using a different computer
inverse_depth = np.array(read_pfm(midas_output_path + "frame.pfm")[0])
midas_depth = np.reciprocal(inverse_depth.copy())

# get MiDaS depth values from pixels with feature points
midas_depths_at_feature_points = []
for row in projected_fp:
    pixel_x = row[0]
    pixel_y = row[1]
    if 0 <= pixel_x < frame.shape[0] and 0 <= pixel_y < frame.shape[1]:
        midas_depths_at_feature_points.append(midas_depth[pixel_x, pixel_y])
    cv.circle(frame, (pixel_x, pixel_y), 5, (51, 14, 247), -1)

# draw circles on the input image
cv.imwrite(f"{data_path}/{user}/{trial}/{frame_number}/featurepoints_{name}.jpg", frame)
cv.imwrite(f"{data_path}/{user}/{trial}/data/featurepoints_{name}.jpg", frame)
# scale LiDAR data
lidar_depth = []
for row in lidar_data:
    x = row[0]* row[3]
    y = row[1] * row[3]
    z = row[2] * row[3]
    lidar_depth.append([x,y,z])

lidar_depth = np.reshape(lidar_depth, (256, 192, 3))[:, :, 2].T * -1

# scale the MiDaS output to the size of the LiDAR depth data
midas_extracted = []
for i in range(lidar_depth.shape[0]):
    for j in range(lidar_depth.shape[1]):
        midas_extracted.append(midas_depth[round(3.75 + 7.5 * i), round(3.75 + 7.5 * j)])
midas_extracted = np.reshape(midas_extracted, (192, 256))

# print out correlations between LiDAR and MiDaS data
correlation = np.corrcoef(np.ravel(lidar_depth), np.ravel(midas_extracted))
spearman = spearmanr(np.ravel(lidar_depth), np.ravel(midas_extracted))
with open(f"{data_path}/{user}/{trial}/{frame_number}/correlation.txt", "w") as f:
    f.write(f"Correlation: {correlation}\nSpearman Correlation: {spearman}")
with open(f"{data_path}/{user}/{trial}/data/correlation.txt", "w") as f:
    f.write(f"Correlation: {correlation}\nSpearman Correlation: {spearman}")
# create a plot comparing LiDAR vs MiDaS and Feature Points vs MiDaS
plt.figure()
plt.scatter(lidar_depth, midas_extracted, label="LiDAR", s=0.5, alpha=0.5)
plt.scatter(ar_depths, midas_depths_at_feature_points, c="r", label="Feature Points")
plt.title("iPhone Depth vs. MiDaS Depth")
plt.legend()
plt.xlabel("iPhone Depth")
plt.ylabel("MiDaS Depth")
plt.savefig(f"{data_path}/{user}/{trial}/{frame_number}/LidarV.Midas_{name}.png")
plt.savefig(f"{data_path}/{user}/{trial}/data/LidarV.Midas_{name}.png")
# create a plot of the LiDAR confidence levels
plt.figure()
plt.pcolor(lidar_confidence, cmap="Greys_r")
plt.colorbar()
plt.title("LiDAR Confidence")
plt.savefig(f"{data_path}/{user}/{trial}/{frame_number}/Confidence_{name}.png")
plt.savefig(f"{data_path}/{user}/{trial}/data/Confidence_{name}.png")
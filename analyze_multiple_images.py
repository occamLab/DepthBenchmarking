"""
Create graphs and data for multiple images at a time.
"""

import os
import shutil
import json
import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv
from scipy.linalg import inv
from scipy.stats import spearmanr
from utils import read_pfm

user = "HccdFYqmqETaJltQbAe19bnyk2e2"
trial = "23CF80B9-5290-4B3B-9EA9-46F083BBE825"
trial_path = "/Users/occamlab/Documents/DepthData/depth_benchmarking/" + \
    user + "/" + trial

midas_input_path = "/Users/occamlab/Documents/ARPointCloud/input"
midas_output_path = "/Users/occamlab/Documents/ARPointCloud/output"

# use: large, hybrid, phone, old
# make sure to also change the weight file in the ./weights directory
weight_used = "phone"

# BEFORE RUNNING, MAKE SURE ALL PATHS AND FILE REFERENCES ARE CORRECT

midas_weights = {"large":("_L", "dpt_large"), "hybrid":("_H", "dpt_hybrid"), \
    "phone":("_P", "midas_v21_small"), "old":("_O", "midas_v21")}

for root, dirs, files in os.walk(trial_path):
    for file in files:
        if file == "frame.jpg":
            new_name = user[0:3] + "_" + trial[0:3] + "_" + root[-4:] + \
                midas_weights[weight_used][0] + ".jpg"
            shutil.copyfile(os.path.join(root, file), os.path.join(midas_input_path, new_name))

os.system("python run.py --model_type " + midas_weights[weight_used][1]) 

for file in os.listdir(midas_output_path):
    name, extension = os.path.splitext(file)
    if extension == ".png" or extension == ".pfm":
        try:
            image_number_dir = name[-6:-2]
            image_number = [int(i) for i in image_number_dir]
            shutil.copyfile(os.path.join(midas_output_path, file), \
                os.path.join(trial_path, image_number_dir, file))
        except:
            continue


if not os.path.exists(os.path.join(trial_path, "data")):
    os.makedirs(os.path.join(trial_path, "data"))

for root, dirs, files in os.walk(trial_path):
    if len(files) >= 4:
        for file in files:
            name, extension = os.path.splitext(file)

            if file == "frame.jpg":
                frame = cv.imread(os.path.join(root, file))

            if extension == ".pfm" and name[-2:] == midas_weights[weight_used][0]:
                inverse_depth = np.array(read_pfm(os.path.join(root, file))[0])

            if file == "framemetadata.json":
                with open(os.path.join(root, file)) as my_file:
                    data = json.load(my_file)

        id = user[0:3] + "_" + trial[0:3] + "_" + root[-4:] + \
                midas_weights[weight_used][0]

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
        
        midas_depth = np.reciprocal(inverse_depth.copy())

        # get MiDaS depth values from pixels with feature points
        midas_depths_at_feature_points = []
        for row in projected_fp:
            pixel_x = row[0]
            pixel_y = row[1]
            if 0 <= pixel_x < frame.shape[0] and 0 <= pixel_y < frame.shape[1]:
                midas_depths_at_feature_points.append(midas_depth[pixel_x, pixel_y])

            cv.circle(frame, (pixel_x, pixel_y), 5, (51, 14, 247), -1)

        cv.imwrite(os.path.join(root, f"fp_{id}.jpg"), frame)
        cv.imwrite(os.path.join(trial_path, "data", f"fp_{id}.jpg"), frame)
        
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

        correlation = np.corrcoef(np.ravel(lidar_depth), np.ravel(midas_extracted))
        spearman = spearmanr(np.ravel(lidar_depth), np.ravel(midas_extracted))
        with open(os.path.join(root, f"corr_{id}.txt"), "w") as text:
            text.write(f"Correlation: {correlation}\nSpearman Correlation: {spearman}")
        with open(os.path.join(trial_path, "data", f"corr_{id}.txt"), "w") as text:
            text.write(f"Correlation: {correlation}\nSpearman Correlation: {spearman}")
        
        # create a plot comparing LiDAR vs MiDaS and Feature Points vs MiDaS
        plt.figure()
        plt.scatter(lidar_depth, midas_extracted, label="LiDAR", s=0.5, alpha=0.5)
        plt.scatter(ar_depths, midas_depths_at_feature_points, c="r", label="Feature Points")
        plt.title("iPhone Depth vs. MiDaS Depth")
        plt.legend()
        plt.xlabel("iPhone Depth")
        plt.ylabel("MiDaS Depth")
        plt.savefig(os.path.join(root, f"scatter_{id}.png"))
        plt.savefig(os.path.join(trial_path, "data", f"scatter_{id}.png"))

        # create a plot of the LiDAR confidence levels
        plt.figure()
        plt.pcolor(lidar_confidence, cmap="RdYlGn")
        plt.colorbar()
        plt.title("LiDAR Confidence")
        plt.savefig(os.path.join(root, f"confidence_{id}.png"))
        plt.savefig(os.path.join(trial_path, "data", f"confidence_{id}.png"))

# deleting used files
for file in os.listdir(midas_input_path):
    name, extension = os.path.splitext(file)
    if extension == ".jpg":
        os.remove(os.path.join(midas_input_path, file))

for file in os.listdir(midas_output_path):
    name, extension = os.path.splitext(file)
    if extension == ".png" or extension == ".pfm":
        os.remove(os.path.join(midas_output_path, file))
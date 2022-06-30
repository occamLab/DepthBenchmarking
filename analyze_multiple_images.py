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

USER = "HccdFYqmqETaJltQbAe19bnyk2e2"
TRIAL = "6CCBDFF7-057E-4FB6-A00F-98E659CE5A88"
TRIAL_PATH = "/Users/occamlab/Documents/DepthData/depth_benchmarking/" + \
    USER + "/" + TRIAL

MIDAS_INPUT_PATH = "/Users/occamlab/Documents/ARPointCloud/input"
MIDAS_OUTPUT_PATH = "/Users/occamlab/Documents/ARPointCloud/output"

# use: large, hybrid, phone, old
# make sure to also change the weight file in the ./weights directory
WEIGHT_USED = "phone"

# BEFORE RUNNING, MAKE SURE ALL PATHS AND FILE REFERENCES ARE CORRECT

midas_weights = {"large":("_L", "dpt_large"), "hybrid":("_H", "dpt_hybrid"), \
    "phone":("_P", "midas_v21_small"), "old":("_O", "midas_v21")}

# delete used files
for file in os.listdir(MIDAS_INPUT_PATH):
    name, extension = os.path.splitext(file)
    if extension == ".jpg":
        os.remove(os.path.join(MIDAS_INPUT_PATH, file))

for file in os.listdir(MIDAS_OUTPUT_PATH):
    name, extension = os.path.splitext(file)
    if extension in (".png", ".pfm"):
        os.remove(os.path.join(MIDAS_OUTPUT_PATH, file))

for root, dirs, files in os.walk(TRIAL_PATH):
    for file in files:
        if file == "frame.jpg":
            new_name = USER[0:3] + "_" + TRIAL[0:3] + "_" + root[-4:] + \
                midas_weights[WEIGHT_USED][0] + ".jpg"
            shutil.copyfile(os.path.join(root, file), os.path.join(MIDAS_INPUT_PATH, new_name))
            frame = cv.imread(os.path.join(MIDAS_INPUT_PATH, new_name))
            cv.imwrite(os.path.join(MIDAS_INPUT_PATH, new_name), cv.rotate(frame, cv.ROTATE_90_CLOCKWISE))

os.system("python run.py --model_type " + midas_weights[WEIGHT_USED][1])

for file in os.listdir(MIDAS_OUTPUT_PATH):
    name, extension = os.path.splitext(file)
    if extension in (".png", ".pfm"):
        try:
            image_number_dir = name[-6:-2]
            image_number = [int(i) for i in image_number_dir]
            shutil.copyfile(os.path.join(MIDAS_OUTPUT_PATH, file), \
                os.path.join(TRIAL_PATH, image_number_dir, file))
        except:
            continue


if not os.path.exists(os.path.join(TRIAL_PATH, "data")):
    os.makedirs(os.path.join(TRIAL_PATH, "data"))

for root, dirs, files in os.walk(TRIAL_PATH):
    if "framemetadata.json" in files:
        for file in files:
            name, extension = os.path.splitext(file)

            if file == "frame.jpg":
                frame = cv.imread(os.path.join(root, file))

            if extension == ".pfm" and name[-2:] == midas_weights[WEIGHT_USED][0]:
                inverse_depth = np.array(read_pfm(os.path.join(root, file))[0])
                inverse_depth = np.rot90(inverse_depth)
                inverse_depth[inverse_depth<1] = np.nan

            if file == "framemetadata.json":
                with open(os.path.join(root, file)) as my_file:
                    data = json.load(my_file)

        tag = USER[0:3] + "_" + TRIAL[0:3] + "_" + root[-4:] + \
                midas_weights[WEIGHT_USED][0]

        # extract data from the JSON file
        raw_fp = np.array(data["rawFeaturePoints"])
        if raw_fp.size == 0:
            continue
        lidar_data = np.array(data["depthData"])
        lidar_confidence = np.reshape(data["confData"], (192, 256))
        pose = np.reshape(data["pose"], (4,4)).T
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
        ar_depths = np.array(ar_depths)

        midas_depth = np.reciprocal(inverse_depth)

        # get MiDaS depth values from pixels with feature points
        midas_depths_at_feature_points = []
        for row in projected_fp:
            pixel_x = row[0]
            pixel_y = row[1]
            if 0 <= pixel_x < frame.shape[0] and 0 <= pixel_y < frame.shape[1]:
                midas_depths_at_feature_points.append(midas_depth[pixel_x, pixel_y])

            cv.circle(frame, (pixel_x, pixel_y), 5, (51, 14, 247), -1)

        midas_depths_at_feature_points = np.array(midas_depths_at_feature_points)

        cv.imwrite(os.path.join(root, f"fp_{tag}.jpg"), frame)
        cv.imwrite(os.path.join(TRIAL_PATH, "data", f"fp_{tag}.jpg"), frame)

        lidar_depth = []
        for row in lidar_data:
            x = row[0] * row[3]
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

        # calculate correlation data
        lidar_midas_correlation = np.corrcoef(np.ravel(lidar_depth[~np.isnan(midas_extracted)]), \
            np.ravel(midas_extracted[~np.isnan(midas_extracted)]))[0][1]
        less_than_five_corr = np.corrcoef(np.ravel(lidar_depth[(lidar_depth<5) & (~np.isnan(midas_extracted))]), \
            np.ravel(midas_extracted[(lidar_depth<5) & (~np.isnan(midas_extracted))]))[0][1]
        mid_high_conf_corr = np.corrcoef(np.ravel(lidar_depth[(lidar_confidence>0) & (~np.isnan(midas_extracted))]), \
            np.ravel(midas_extracted[(lidar_confidence>0) & (~np.isnan(midas_extracted))]))[0][1]
        high_conf_corr = np.corrcoef(np.ravel(lidar_depth[(lidar_confidence==2) & (~np.isnan(midas_extracted))]), \
            np.ravel(midas_extracted[(lidar_confidence==2) & (~np.isnan(midas_extracted))]))[0][1]
        lidar_midas_spearman = spearmanr(np.ravel(lidar_depth[~np.isnan(midas_extracted)]), \
            np.ravel(midas_extracted[~np.isnan(midas_extracted)]))
        ar_midas_corr = np.corrcoef(np.ravel(ar_depths[~np.isnan(midas_depths_at_feature_points)]), \
            np.ravel(midas_depths_at_feature_points[~np.isnan(midas_depths_at_feature_points)]))[0][1]
        ar_midas_spearman = spearmanr(np.ravel(ar_depths[~np.isnan(midas_depths_at_feature_points)]), \
            np.ravel(midas_depths_at_feature_points[~np.isnan(midas_depths_at_feature_points)]))

        # write the correlation data into a text file
        with open(os.path.join(root, f"corr_{tag}.txt"), "w") as text:
            text.write(f"LIDAR/MIDAS CORRELATIONS\nCorrelation: {lidar_midas_correlation}\n" \
                f"Correlation for LiDAR < 5m: {less_than_five_corr}\n" \
                f"Correlation for mid-high LiDAR confidence: {mid_high_conf_corr}\n" \
                f"Correlation for high LiDAR confidence: {high_conf_corr}\n" \
                f"Spearman correlation: {lidar_midas_spearman}\n\n" \
                f"ARKIT/MIDAS CORRELATIONS\nCorrelation: {ar_midas_corr}\n" \
                f"Spearman correlation: {ar_midas_spearman}")
        shutil.copyfile(os.path.join(root, f"corr_{tag}.txt"), \
            os.path.join(TRIAL_PATH, "data", f"corr_{tag}.txt"))

        # create a plot comparing LiDAR vs MiDaS and Feature Points vs MiDaS
        plt.figure()
        plt.scatter(lidar_depth, midas_extracted, label="LiDAR", s=0.5, alpha=0.5)
        plt.scatter(ar_depths, midas_depths_at_feature_points, c="r", label="Feature Points")
        plt.title("iPhone Depth vs. MiDaS Depth")
        plt.legend()
        plt.xlabel("iPhone Depth")
        plt.ylabel("MiDaS Depth")
        plt.savefig(os.path.join(root, f"scatter_{tag}.png"))
        plt.savefig(os.path.join(TRIAL_PATH, "data", f"scatter_{tag}.png"))
        plt.close()

        # create a plot of the LiDAR confidence levels
        plt.figure()
        plt.pcolor(lidar_confidence, cmap="RdYlGn")
        plt.colorbar()
        plt.title("LiDAR Confidence")
        plt.savefig(os.path.join(root, f"confidence_{tag}.png"))
        plt.savefig(os.path.join(TRIAL_PATH, "data", f"confidence_{tag}.png"))
        plt.close()

        # create a plot of LiDAR and Midas correlation for only high confidence points 
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
        plt.savefig(os.path.join(root, f"high_conf_corr_{tag}.png"))
        plt.savefig(os.path.join(TRIAL_PATH, "data", f"high_conf_corr_{tag}.png"))
        plt.close()
        
        # create a plot of LiDAR and Midas correlation for distances less than five meters 
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
        plt.savefig(os.path.join(root, f"less_five_corr_{tag}.png"))
        plt.savefig(os.path.join(TRIAL_PATH, "data", f"less_five_corr_{tag}.png"))
        plt.close()

# delete used files
for file in os.listdir(MIDAS_INPUT_PATH):
    name, extension = os.path.splitext(file)
    if extension == ".jpg":
        os.remove(os.path.join(MIDAS_INPUT_PATH, file))

for file in os.listdir(MIDAS_OUTPUT_PATH):
    name, extension = os.path.splitext(file)
    if extension in (".png", ".pfm"):
        os.remove(os.path.join(MIDAS_OUTPUT_PATH, file))

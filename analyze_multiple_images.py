"""
Create graphs and data for multiple images at a time.
"""

import os
import shutil
import json
import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv
from sklearn.linear_model import RANSACRegressor
from math import floor
from scipy.linalg import inv
from scipy.stats import spearmanr
from utils import read_pfm
from Mesh_pb2 import Points

USER = "HccdFYqmqETaJltQbAe19bnyk2e2"
TRIAL = "F231CF41-FBCE-4C5B-8E88-BD05FEDB7591"
TRIAL_PATH = "/Users/occamlab/Documents/DepthData/depth_benchmarking/" + \
    USER + "/" + TRIAL

MIDAS_INPUT_PATH = "/Users/occamlab/Documents/ARPointCloud/input"
MIDAS_OUTPUT_PATH = "/Users/occamlab/Documents/ARPointCloud/output"

# use: large, hybrid, phone, old
# make sure to also change the weight file in the ./weights directory
WEIGHT_USED = "phone"

RUN_MIDAS = True

# BEFORE RUNNING, MAKE SURE ALL PATHS AND FILE REFERENCES ARE CORRECT

midas_weights = {"large":("_L", "dpt_large"), "hybrid":("_H", "dpt_hybrid"), \
    "phone":("_P", "midas_v21_small"), "old":("_O", "midas_v21")}

if RUN_MIDAS:
    # delete used files
    for file in os.listdir(MIDAS_INPUT_PATH):
        name, extension = os.path.splitext(file)
        if extension == ".jpg":
            os.remove(os.path.join(MIDAS_INPUT_PATH, file))

    for file in os.listdir(MIDAS_OUTPUT_PATH):
        name, extension = os.path.splitext(file)
        if extension in (".png", ".pfm"):
            os.remove(os.path.join(MIDAS_OUTPUT_PATH, file))

    # copy the rotated version of the images into the MiDaS input folder
    for root, dirs, files in os.walk(TRIAL_PATH):
        for file in files:
            if file == "frame.jpg":
                new_name = USER[0:3] + "_" + TRIAL[0:3] + "_" + root[-4:] + \
                    midas_weights[WEIGHT_USED][0] + ".jpg"
                shutil.copyfile(os.path.join(root, file), os.path.join(MIDAS_INPUT_PATH, new_name))
                frame = cv.imread(os.path.join(MIDAS_INPUT_PATH, new_name))
                cv.imwrite(os.path.join(MIDAS_INPUT_PATH, new_name), cv.rotate(frame, cv.ROTATE_90_CLOCKWISE))

    # run MiDaS
    os.system("python run.py --model_type " + midas_weights[WEIGHT_USED][1])

    # copy the PNG and PFM files from the output 
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

# create a data folder in the trial folder if it doesn't exist
if os.path.exists(TRIAL_PATH) and not os.path.exists(os.path.join(TRIAL_PATH, "data")):
    os.makedirs(os.path.join(TRIAL_PATH, "data"))

for root, dirs, files in os.walk(TRIAL_PATH):
    # only run analysis if the JSON file with the metadata is in the folder
    if "framemetadata.json" in files and "pointcloud.pb" in files:
        for file in files:
            name, extension = os.path.splitext(file)

            # read the image captured by the camera
            if file == "frame.jpg":
                frame = cv.imread(os.path.join(root, file))

            # read the PFM file
            if extension == ".pfm" and name[-2:] == midas_weights[WEIGHT_USED][0]:
                inverse_depth = np.array(read_pfm(os.path.join(root, file))[0])
                # rotate PFM data to match image read above
                inverse_depth = np.rot90(inverse_depth)
                # remove some outliers
                inverse_depth[inverse_depth<1] = np.nan

            # load the data from the JSON file
            if file == "framemetadata.json":
                with open(os.path.join(root, file)) as my_file:
                    data = json.load(my_file)
            
            if file == "pointcloud.pb":
                with open("pointcloud.pb", "rb") as my_file:
                    cloud = Points()
                    cloud.ParseFromString(my_file.read())


        # create a unique ID tag for the current trial and model of MiDaS run
        tag = USER[0:3] + "_" + TRIAL[0:3] + "_" + root[-4:] + \
                midas_weights[WEIGHT_USED][0]

        # extract data from the JSON file
        raw_fp = np.array(data["rawFeaturePoints"])
        if raw_fp.size == 0:
            continue
        pose = np.reshape(data["pose"], (4,4)).T
        raw_fp = np.hstack((raw_fp, np.ones((raw_fp.shape[0], 1)))).T
        projected_fp = np.around(data["projectedFeaturePoints"]).astype(int)
        focal_length = data["intrinsics"][0]
        offset_x = data["intrinsics"][6]
        offset_y = data["intrinsics"][7]

        lidar_depth = []
        for point in cloud.points:
            lidar_depth.append([point.u * point.d, point.v * point.d, point.w * point.d])
        lidar_depth = np.array(lidar_depth)
        lidar_confidence = np.reshape(cloud.confidences, (192, 256))

        # translate feature points to other coordinate frames
        phone_fp = inv(pose) @ raw_fp
        camera_fp = np.array((phone_fp[0], -phone_fp[1], -phone_fp[2])).T

        # calculate depths of feature points
        ar_depths = []
        for row in camera_fp:
            pixel_col = row[0] * focal_length / row[2] + offset_x + 0.5
            pixel_row = row[1] * focal_length / row[2] + offset_y + 0.5
            if 0 <= round(pixel_col) < frame.shape[1] \
                and 0 <= round(pixel_row) < frame.shape[0]:
                ar_depths.append(row[2])
        ar_depths = np.array(ar_depths)

        # get the MiDaS depth by taking the reciprocal of the MiDaS output
        midas_depth = np.reciprocal(inverse_depth)

        # save LiDAR point cloud to csv file
        lidar_depth = np.array(lidar_depth)
        np.savetxt((os.path.join(root, f"lidar_depth.csv")), \
            np.hstack((lidar_depth, np.ravel(lidar_confidence)[:, None])), \
            delimiter=",")

        # extract depth from the properly scaled LiDAR data
        lidar_depth = np.reshape(lidar_depth, (192, 256, 3))[:, :, 2] * -1

        # get MiDaS and LiDAR depth values from pixels with feature points
        midas_depths_at_feature_points = []
        lidar_depths_at_feature_points = []
        lidar_confidence_at_feature_points = []
        for row in projected_fp:
            pixel_col = row[0]
            pixel_row = row[1]
            if 0 <= pixel_col < frame.shape[1] and 0 <= pixel_row < frame.shape[0]:
                midas_depths_at_feature_points.append(midas_depth[pixel_row, pixel_col])
                lidar_depths_at_feature_points.append(lidar_depth\
                    [floor(pixel_row / 7.5), floor(pixel_col / 7.5)])
                lidar_confidence_at_feature_points.append(lidar_confidence\
                    [floor(pixel_row / 7.5), floor(pixel_col / 7.5)])
                inverse_color = tuple(int(x) for x in (255 - frame[pixel_row][pixel_col]))
                # draw circles and numbers to mark feature points on the image
                cv.circle(frame, (pixel_col, pixel_row), 5, (0, 255, 0), -1)
                cv.putText(frame, str(len(midas_depths_at_feature_points)), \
                    (pixel_col, pixel_row), cv.FONT_HERSHEY_COMPLEX, 1, inverse_color)

        midas_depths_at_feature_points = np.array(midas_depths_at_feature_points)
        lidar_depths_at_feature_points = np.array(lidar_depths_at_feature_points)
        lidar_confidence_at_feature_points = np.array(lidar_confidence_at_feature_points)

        # remove more outliers
        if midas_depths_at_feature_points.size > 0:
            midas_depth[midas_depth>=2*max(midas_depths_at_feature_points)] = np.nan
            
        # save the image marked with feature points
        cv.imwrite(os.path.join(root, f"fp_{tag}.jpg"), frame)
        cv.imwrite(os.path.join(TRIAL_PATH, "data", f"fp_{tag}.jpg"), frame)

        # scale the MiDaS output to the size of the LiDAR depth data
        midas_extracted = []
        for i in range(lidar_depth.shape[0]):
            for j in range(lidar_depth.shape[1]):
                midas_extracted.append(midas_depth[round(3.75 + 7.5 * i), round(3.75 + 7.5 * j)])
        midas_extracted = np.reshape(midas_extracted, (192, 256))

        # calculate correlations
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

        plt.figure()
        plt.scatter(ar_depths, lidar_depths_at_feature_points)
        plt.xlabel("Feature Points")
        plt.ylabel("LiDAR")
        plt.title("Feature Points vs All LiDAR")
        for i in range(ar_depths.size):
            plt.annotate(str(i), (ar_depths[i], lidar_depths_at_feature_points[i]))
        plt.savefig(os.path.join(root, f"lidar_fp_corr_{tag}.png"))
        plt.savefig(os.path.join(TRIAL_PATH, "data", f"lidar_fp_corr_{tag}.png"))
        plt.close()

        # calculate line of best fit
        if len(ar_depths[~np.isnan(midas_depths_at_feature_points)]) > 10:
            valid_midas_at_fp = midas_depths_at_feature_points[~np.isnan(midas_depths_at_feature_points)]
            A = np.vstack([valid_midas_at_fp.ravel(), np.ones(valid_midas_at_fp.size)]).T

            ransac = RANSACRegressor(max_trials=300)
            ransac.fit(A, ar_depths[~np.isnan(midas_depths_at_feature_points)].ravel())
            ransac_prediction = ransac.predict(A)

            m, b = np.linalg.lstsq(A, ransac_prediction, rcond=None)[0]
            m2, b2 = np.linalg.lstsq(A, ar_depths[~np.isnan(midas_depths_at_feature_points)], rcond=None)[0]

            #plot feature points vs midas
            plt.figure()
            plt.plot(valid_midas_at_fp, ar_depths[~np.isnan(midas_depths_at_feature_points)], \
                'o', label='Original data', markersize=5)
            plt.plot(valid_midas_at_fp, valid_midas_at_fp * m2 + b2, c="orange", label="Lstsq")
            plt.plot(valid_midas_at_fp, ransac_prediction, c="r", label="RANSAC")
            plt.xlabel("Midas Relative Depth")
            plt.ylabel("AR depth (m)")
            plt.title("Midas Relative Depth v. AR Depth")
            plt.legend()
            plt.savefig(os.path.join(root, f"linear_fit_midas_ar_{tag}.png"))
            plt.savefig(os.path.join(TRIAL_PATH, "data", f"linear_fit_midas_ar_{tag}.png"))
            plt.close()
            midas_absolute = m * midas_extracted + b
            # create midas point cloud
            midas_point_cloud = []
            for pixel_row in range(midas_absolute.shape[0]):
                for pixel_col in range(midas_absolute.shape[1]):
                    x = (pixel_col * 7.5 + 3.75 - offset_x - 0.5) * midas_absolute[pixel_row][pixel_col] / focal_length
                    y = (pixel_row * 7.5 + 3.75 - offset_y - 0.5) * midas_absolute[pixel_row][pixel_col] / focal_length
                    if midas_absolute[pixel_row][pixel_col] < 20:
                        midas_point_cloud.append((x, -y, -midas_absolute[pixel_row][pixel_col]))
            # save midas point cloud to cvs file
            if m > 0:
                np.savetxt(os.path.join(root, f"midas_point_cloud.csv"), midas_point_cloud, delimiter=",")

# delete used files
for file in os.listdir(MIDAS_INPUT_PATH):
    name, extension = os.path.splitext(file)
    if extension == ".jpg":
        os.remove(os.path.join(MIDAS_INPUT_PATH, file))

for file in os.listdir(MIDAS_OUTPUT_PATH):
    name, extension = os.path.splitext(file)
    if extension in (".png", ".pfm"):
        os.remove(os.path.join(MIDAS_OUTPUT_PATH, file))

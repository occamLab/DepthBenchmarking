"""
This file calculates the mean values of the correlation from the text files of a trial. The trial
needs to have text files from large and phone version of Midas.
"""
import os
import numpy as np

# change based on computer path: Data is a folder with all proccesed data trials
DIRECTORY = "/Users/angrocki/Desktop/DepthData/depth_benchmarking/"
USER = "HccdFYqmqETaJltQbAe19bnyk2e2"
TRIAL = "FCABFB69-BB71-491D-8AC8-9B1D20F83B62"
path = os.path.join(DIRECTORY, f"{USER}/{TRIAL}/data/")

# varibles for large Midas
lidar_midas_correlation_L = []
less_than_five_corr_L = []
mid_high_conf_corr_L = []
high_conf_corr_L = []
lidar_midas_spearman_L = []
ar_midas_corr_L = []
ar_midas_spearman_L = []

# variables for phone midas
lidar_midas_correlation_P = []
less_than_five_corr_P = []
mid_high_conf_corr_P = []
high_conf_corr_P = []
lidar_midas_spearman_P = []
ar_midas_corr_P = []
ar_midas_spearman_P = []

# collect data from text files
for file in os.listdir(path):
    name, extension = os.path.splitext(file)
    if extension == ".txt" and name[-1:] == "L":
        var = {}
        with open(f"{path}{file}") as myfile:
            lines = myfile.readlines()
            lidar_midas_correlation_L.append(float(lines[1].split(": ")[1]))
            less_than_five_corr_L.append(float(lines[2].split(": ")[1]))
            mid_high_conf_corr_L.append(float(lines[3].split(": ")[1]))
            high_conf_corr_L.append(float(lines[4].split(": ")[1]))
            ar_midas_corr_L.append(float(lines[8].split(": ")[1]))
            lidar_midas_value = lines[5].split(": ")[1].split("=")[1].split(",")[0]
            lidar_midas_spearman_L.append(float(lidar_midas_value))
            ar_midas_value = lines[9].split(": ")[1].split("=")[1].split(",")[0]
            ar_midas_spearman_L.append(float(ar_midas_value))
    if extension == ".txt" and name[-1:] == "P":
        var = {}
        with open(f"{path}{file}") as myfile:
            lines = myfile.readlines()
            lidar_midas_correlation_P.append(float(lines[1].split(": ")[1]))
            less_than_five_corr_P.append(float(lines[2].split(": ")[1]))
            mid_high_conf_corr_P.append(float(lines[3].split(": ")[1]))
            high_conf_corr_P.append(float(lines[4].split(": ")[1]))
            ar_midas_corr_P.append(float(lines[8].split(": ")[1]))
            lidar_midas_value = lines[5].split(": ")[1].split("=")[1].split(",")[0]
            lidar_midas_spearman_P.append(float(lidar_midas_value))
            ar_midas_value = lines[9].split(": ")[1].split("=")[1].split(",")[0]
            ar_midas_spearman_P.append(float(ar_midas_value))

# Calculating means
mean_lidar_midas_correlation_L = np.nanmean(lidar_midas_correlation_L)
mean_less_than_five_corr_L = np.nanmean(less_than_five_corr_L)
mean_mid_high_conf_corr_L = np.nanmean(mid_high_conf_corr_L)
mean_high_conf_corr_L = np.nanmean(high_conf_corr_L)
mean_lidar_midas_spearman_L = np.mean(lidar_midas_spearman_L)
mean_ar_midas_corr_L = np.nanmean(ar_midas_corr_L)
mean_ar_midas_spearman_L = np.mean(ar_midas_spearman_L)

mean_lidar_midas_correlation_P = np.nanmean(lidar_midas_correlation_P)
mean_less_than_five_corr_P = np.nanmean(less_than_five_corr_P)
mean_mid_high_conf_corr_P = np.nanmean(mid_high_conf_corr_P)
mean_high_conf_corr_P = np.nanmean(high_conf_corr_P)
mean_lidar_midas_spearman_P = np.nanmean(lidar_midas_spearman_P)
mean_ar_midas_corr_P = np.nanmean(ar_midas_corr_P)
mean_ar_midas_spearman_P = np.nanmean(ar_midas_spearman_P)

# write into a text file
with open(os.path.join(path, f"mean_corr_{TRIAL[0:3]}.txt"), "w") as text:
    text.write(f"MEAN FOR LARGE MIDAS\nLIDAR/MIDAS CORRELATIONS\n" \
        f"Correlation: {mean_lidar_midas_correlation_L}\n" \
        f"Correlation for LiDAR < 5m: {mean_less_than_five_corr_L}\n" \
        f"Correlation for mid-high LiDAR confidence: {mean_mid_high_conf_corr_L}\n" \
        f"Correlation for high LiDAR confidence: {mean_high_conf_corr_L}\n" \
        f"Spearman correlation: {mean_lidar_midas_spearman_L}\n" \
        f"ARKIT/MIDAS CORRELATIONS\nCorrelation: {mean_ar_midas_corr_L}\n" \
        f"Spearman correlation: {mean_ar_midas_spearman_L}\n\n" \
        f"MEAN FOR PHONE MIDAS\nLIDAR/MIDAS CORRELATIONS\n" \
        f"Correlation: {mean_lidar_midas_correlation_P}\n" \
        f"Correlation for LiDAR < 5m: {mean_less_than_five_corr_P}\n" \
        f"Correlation for mid-high LiDAR confidence: {mean_mid_high_conf_corr_P}\n" \
        f"Correlation for high LiDAR confidence: {mean_high_conf_corr_P}\n" \
        f"Spearman correlation: {mean_lidar_midas_spearman_P}\n" \
        f"ARKIT/MIDAS CORRELATIONS\nCorrelation: {mean_ar_midas_corr_P}\n" \
        f"Spearman correlation: {mean_ar_midas_spearman_P}\n" )

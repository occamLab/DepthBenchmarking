import os
import numpy as np

# change based on computer path: Data is a folder with all proccesed data trails
directory = "/Users/occamlab/Documents/DepthData/depth_benchmarking/HccdFYqmqETaJltQbAe19bnyk2e2"
trial = "7C271404-B251-450E-9C52-DB3C6BDF448D"
path = os.path.join(directory, f"{trial}/data/")

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
            lidar_midas_spearman_L.append(lines[5].split(": ")[1])
            ar_midas_corr_L.append(float(lines[8].split(": ")[1]))
            ar_midas_spearman_L.append(lines[9].split(": ")[1])
    if extension == ".txt" and name[-1:] == "P":
        var = {}
        with open(f"{path}{file}") as myfile:
            lines = myfile.readlines()
            lidar_midas_correlation_P.append(float(lines[1].split(": ")[1]))
            less_than_five_corr_P.append(float(lines[2].split(": ")[1]))
            mid_high_conf_corr_P.append(float(lines[3].split(": ")[1])) 
            high_conf_corr_P.append(float(lines[4].split(": ")[1]))
            lidar_midas_spearman_P.append(lines[5].split(": ")[1])
            ar_midas_corr_P.append(float(lines[8].split(": ")[1]))
            ar_midas_spearman_P.append(lines[9].split(": ")[1])

# Calculating means
mean_lidar_midas_correlation_L = np.nanmean(lidar_midas_correlation_L)
mean_less_than_five_corr_L = np.nanmean(less_than_five_corr_L)
mean_mid_high_conf_corr_L = np.nanmean(mid_high_conf_corr_L)
mean_high_conf_corr_L = np.nanmean(high_conf_corr_L)
#mean_lidar_midas_spearman_L = 
mean_ar_midas_corr_L = np.nanmean(ar_midas_corr_L)
#mean_ar_midas_spearman_L = []     

mean_lidar_midas_correlation_P = np.nanmean(lidar_midas_correlation_P)
mean_less_than_five_corr_P = np.nanmean(less_than_five_corr_P)
mean_mid_high_conf_corr_P = np.nanmean(mid_high_conf_corr_P)
mean_high_conf_corr_P = np.nanmean(high_conf_corr_P)
#mean_lidar_midas_spearman_P =
mean_ar_midas_corr_P = np.nanmean(ar_midas_corr_P)
#mean_ar_midas_spearman_P =    
                
# write into a text file
with open(os.path.join(path, f"mean_corr_{trial[0:3]}.txt"), "w") as text:
    text.write(f"MEAN FOR LARGE MIDAS\nLIDAR/MIDAS CORRELATIONS\nCorrelation: {mean_lidar_midas_correlation_L}\n" \
        f"Correlation for LiDAR < 5m: {mean_less_than_five_corr_L}\n" \
        f"Correlation for mid-high LiDAR confidence: {mean_mid_high_conf_corr_L}\n" \
        f"Correlation for high LiDAR confidence: {mean_high_conf_corr_L}\n" \
        f"ARKIT/MIDAS CORRELATIONS\nCorrelation: {mean_ar_midas_corr_L}\n" \
        f"MEAN FOR PHONE MIDAS\nLIDAR/MIDAS CORRELATIONS\nCorrelation: {mean_lidar_midas_correlation_P}\n" \
        f"Correlation for LiDAR < 5m: {mean_less_than_five_corr_P}\n" \
        f"Correlation for mid-high LiDAR confidence: {mean_mid_high_conf_corr_P}\n" \
        f"Correlation for high LiDAR confidence: {mean_high_conf_corr_P}\n" \
        f"ARKIT/MIDAS CORRELATIONS\nCorrelation: {mean_ar_midas_corr_P}\n" )
"""
Create graphs and data for multiple images at a time.
"""

import os
import shutil
import json
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import spearmanr
from utils import read_pfm

user = "HccdFYqmqETaJltQbAe19bnyk2e2"
trial = "23CF80B9-5290-4B3B-9EA9-46F083BBE825"
# "6CCBDFF7-057E-4FB6-A00F-98E659CE5A88"
path = "/Users/occamlab/Documents/DepthData/depth_benchmarking/" + user + "/" + trial

midas_input_path = "/Users/occamlab/Documents/ARPointCloud/input"

for root, dirs, files in os.walk(path):
    for file in files:
        name, extension = os.path.splitext(file)
        if extension.lower() == ".jpg":
            new_name = user[0:3] + "_" + trial[0:3] + "_" + root[-4:] + extension
            new_path = os.path.join(root, new_name)
            if name == "frame":
                os.rename(os.path.join(root, file), new_path)
            shutil.copyfile(new_path, os.path.join(midas_input_path, new_name))

exec("run.py")
# run midas on all of the images
# grab the images from the output folder and put them into the right folders

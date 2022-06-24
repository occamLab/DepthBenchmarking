"""
Create graphs and data for multiple images at a time.
"""

import os
import shutil

user = "HccdFYqmqETaJltQbAe19bnyk2e2"
trial = "E413ADD0-0D92-4D42-9467-997ABC59F5F4"
trial_path = "/Users/occamlab/Documents/DepthData/depth_benchmarking/" + \
    user + "/" + trial

midas_input_path = "/Users/occamlab/Documents/ARPointCloud/input"
midas_output_path = "/Users/occamlab/Documents/ARPointCloud/output"

# use: large, hybrid, phone, old
# make sure to also change the weight file in the ./weights directory
weight_used = "large"

# BEFORE RUNNING, MAKE SURE ALL PATHS AND FILE REFERENCES ARE CORRECT

midas_weights = {"large":("_L", "dpt_large"), "hybrid":("_H", "dpt_hybrid"), \
    "phone":("_P", "midas_v21_small"), "old":("_O", "midas_v21")}

for root, dirs, files in os.walk(trial_path):
    for file in files:
        name, extension = os.path.splitext(file)
        if extension == ".jpg":
            new_name = user[0:3] + "_" + trial[0:3] + "_" + root[-4:] + \
                midas_weights[weight_used][0] + extension
            new_path = os.path.join(root, new_name)
            if name == "frame":
                os.rename(os.path.join(root, file), new_path)
            shutil.copyfile(new_path, os.path.join(midas_input_path, new_name))

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

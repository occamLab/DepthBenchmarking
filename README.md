
# Monocular Depth Prediction Benchmarking

The purpose of this project is to analyze the performance of MiDaS depth prediction so it can be integrated into an iPhone app that determines the depth of objects in the camera frame. It includes the logic for scaling the output of MiDaS to depth in meters using Apple's ARKit feature points and analysis of point clouds to find obstacles in the camera frame. This project includes an app that collects and uploads image data (ARPoints), parts of [MiDaS neural network depth prediction](https://github.com/isl-org/MiDaS), and benchmarking and analysis files for depth prediction.

## ARPoints

The `ARPoints` folder contains and Xcode project for an iPhone app called DepthBenchmarking used to collect data while navigating an area. This app gives the user a video feed of the iPhone camera, along with visualizations of ARKit feature points in the frame. The camera frames and metadata are captured and uploaded to FireBase at a constant time interval, and we can later download this data to analyze it using other files in this repository. The uploaded data for each frame includes a JPEG of the captured image, a JSON file containing iPhone intrinsics, iPhone pose, and location of feature points, and a protobuf containing LiDAR data. Our app was built off of [this app](https://github.com/BlackMirrorz/ARPointCloud) that simply visualized the feature points in the frame.

**Note: This app can only collect sufficient data on iPhones equipped with LiDAR. The user must also have access to OCCaMLab's FireBase database.**

## MiDaS

In order to test the accuracy of monocular depth prediction using the MiDaS neural network, we integrated the framework into this repository. The `midas` folder, along with `run.py` and `utils.py` are adopted from the [original MiDaS repo](https://github.com/isl-org/MiDaS). To use MiDaS, first follow the steps under *Setup* in the original MiDaS repo to download the desired model (we used midas_v21_small) and install the required packages. Then, follow the *Usage* section to run the model on sample images. Read below for more detailed steps on analyzing one or multiple images at a time.

## Package Dependencies

In addition to the packages required to run MiDaS, the Python files for analysis (`analyze_one_image.py` and `analyze_multiple_images.py`) require the installation of a number of packages to run. The installation commands are listed below:

```
pip3 install numpy
pip3 install matplotlib
pip3 install opencv-python
pip3 install scipy
pip3 install open3d
pip3 install sklearn
```

## Analyzing a Single Frame

1) Start by running MiDaS on an image that is in portrait orientation. The image should be 1920x1440 pixels. Leave the image in the `input` folder. To make sure that MiDaS ran, check that `frame.pfm` and `frame.png` are in the `output` folder.

2) Make sure `pointcloud.pb` and `framemetadata.json` for the image you are analyzing are in the main directory of the repo.

3) Run `analyze_one_image.py`. The first figure that pops up is a 3D point cloud of the LiDAR data. After closing that, a 3D point cloud of the MiDaS data will appear. Finally, a number of Matplotlib figures of various visuals will appear. Some of the visuals are commented out in the code but can be added to the output if the user removes the comment.

4) Run `get_point_clouds.m` to view various point clouds of both LiDAR and MiDaS data in MATLAB. This file extracts data from `lidar_depth.csv` and `midas_point_cloud.csv` in the repo's main directory, so it is necessary to do step 3 first so that those files are updated.

**Note: As is, `get_point_clouds.m` can be run without running `analyze_one_image.py` because we have included a sample image (`frame.jpg`) and the accompanying CSV files. The image can be found in the `input` folder.**

## Analyzing All Images in a Trial

1) Modify `analyze_multiple_images.py` so that all of the directories at the top of the file match those of the current user's computer. The frames and accompanying data downloaded from FireBase should be stored in such a way where the directory progression is `mainFolder` -> `userID` -> `trialID` -> `imageNumber`. 

2) Run `analyze_multiple_images.py`. This performs much of the same analysis as `analyze_one_image.py` and saves all of the MiDaS output and the Matplotlib figures in the correct image folders. It also creates a directory in the trial called `data` that contains copies of figures from all of the images.

3) Run `get_correlation_means.py` (after changing the directory names appropriately) to get an overview of how iPhone and MiDaS depth data correlate accross all images in the trial.

4) Run `get_all_point_clouds.m` in MATLAB to save MiDaS and LiDAR comparison point clouds and histograms helpful for object detection in each image folder.
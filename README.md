
# Monocular Depth Prediction Benchmarking

This project is includes an app that collects and uploads image data (ARPoints), parts of [MiDaS neural network depth prediction](https://github.com/isl-org/MiDaS), and benchmarking and analysis files for depth prediction.

## ARPoints

The `ARPoints` folder contains and XCode project for an iPhone app called DepthBenchmarking used to collect data while navigating an area. This app gives the user a video feed of the iPhone camera, along with visualizations of ARKit feature points in the frame. The camera frames and metadata are captured and uploaded to FireBase at a constant time interval, and we can later download this data to analyze it using other files in this repository. The uploaded data for each frame includes a JPEG of the captured image, a JSON file containing iPhone intrinsics, iPhone pose, and location of feature points, and a protobuf containing LiDAR data. Our app was built off of [this app](https://github.com/BlackMirrorz/ARPointCloud) that simply visualized the feature points in the frame.

**Note: This app can only collect sufficient data on iPhones equipped with LiDAR. The user must also have access to OCCaMLab's FireBase database.**

## MiDaS

In order to test the accuracy of monocular depth prediction using the MiDaS neural network, we integrated the framework into this repository. The `midas` folder, along with `run.py` and `utils.py` are adopted from the [original MiDaS repo] (https://github.com/isl-org/MiDaS). To use MiDaS, first follow the steps under *Setup* in the original MiDaS repo to download the desired model (we used midas_v21_small) and install the required packages. Then, follow the *Usage* section to run the model on sample images. Read below for more detailed steps for analyzing one or multiple images at a time.

## Package Dependencies
In addition to the packages required to run MiDaS, the Python files for analysis (`analyze_one_image.py` and `analyze_multiple_images.py`) require the installation of a number of packages to run. The installation commands are listed below:

`pip3 install numpy`
`pip3 install matplotlib`
`pip3 install opencv-python`
`pip3 install sklearn`
`pip3 install scipy`

## Analyzing a Single Frame

## Analyzing All Images in a Trial
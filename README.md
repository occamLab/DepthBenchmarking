
# Monocular Depth Prediction Benchmarking

This project is includes an app that collects and uploads image data (ARPoints), parts of [MiDaS neural network depth prediction](https://github.com/isl-org/MiDaS), and benchmarking and analysis files for depth prediction.

## ARPoints

The ARPoints folder contains and XCode project for an iPhone app called DepthBenchmarking used to collect data while navigating an area. This app gives the user a video feed of the iPhone camera, along with visualizations of ARKit feature points in the frame. The camera frames and metadata are captured and uploaded to FireBase at a constant time interval, and we can later download this data to analyze it using other files in this repository. The uploaded data for each frame includes a JPEG of the captured image, a JSON file containing iPhone intrinsics, iPhone pose, and location of feature points, and a protobuf containing LiDAR data. 

**Note: this app can only collect sufficient data on iPhones equipped with LiDAR.**

## MiDaS

## Analyzing a Single Frame

## Analyzing All Images in a Trial

## Sample Data

## Package Dependencies
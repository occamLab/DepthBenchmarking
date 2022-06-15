"""
This file loads a captured frame and associated metadata and marks the feature
points on the image.
"""

import json
import numpy as np
import cv2 as cv

frame = cv.imread("frame.jpg")

with open("framemetadata.json") as f:
    data = json.load(f)
    lidarDepth = np.array(data["depthData"])
    pose = np.reshape(data["pose"], (4,4))
    rawFP = np.array(data["rawFeaturePoints"])
    projectedFP = np.array(data["projectedFeaturePoints"])

print(pose)

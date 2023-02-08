from vision_pipeline import VisionPipeline
import pyrealsense2 as rs

import numpy as np
import time
import sys
from cv2 import aruco
from scipy.spatial.transform import Rotation
import pandas as pd


DEBUG = 1


if __name__ == '__main__':
    sys.setrecursionlimit(1000000)

    depth_res=(720, 1280)
    rgb_res=(1080, 1920)
    align_to="rgb"
    marker_size=3.62 #13.8
    marker_type=aruco.DICT_4X4_50
    required_marker_id = 6

    pipeline = VisionPipeline(depth_res, rgb_res, align_to, marker_size, marker_type, required_marker_id, debug=DEBUG)
    temp_queue = []
    
    while True:
        pipeline.cam_process(temp_queue)
        temp_queue.clear()
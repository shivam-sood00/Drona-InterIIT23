import pyrealsense2 as rs
import numpy as np
import cv2
from cv2 import aruco
import time
import os
from scipy.spatial.transform import Rotation
import csv

# pipeline = rs.pipeline()
# config = rs.config()
# config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 250)
# #config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
# profile = pipeline.start(config)
# frames = pipeline.wait_for_frames()
# for frame in frames:
    # motion_data = frame.as_motion_frame().get_motion_data()
    # print(motion_data)
    # prints: x: -0.0294199, y: -7.21769, z: -6.41355 for me
    # to get numpy array:
accel_param = np.array([0.029, 0.078, -9.356]).reshape(-1,3)
accel_param_norm  = np.linalg.norm(accel_param)
req_accel = np.array([0,0,-accel_param_norm]).reshape(-1,3)
rotated_vec = Rotation.align_vectors(req_accel,accel_param)
eul = rotated_vec[0].as_euler("xyz")
eul[2] = 0
# r = Rotation.from_euler("xyz", eul, degrees=False)
# rot_mat = r.as_matrix()
print(eul)

import pyrealsense2 as rs
import numpy as np
from scipy.spatial.transform import Rotation

'''
Utility Script for camera roll and pitch callibration using IMU data of Realsense 
'''

# Initialize Realsense
# pipeline = rs.pipeline()

# # Get camera default configuration
# config = rs.config()

# # Enable stream of IMU Data
# config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 250)
# #config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)

# # Start the pipeline and wait for the data
# profile = pipeline.start(config)
# frames = pipeline.wait_for_frames()

# for frame in frames:
    # Get the IMU Data
# motion_data = frame.as_motion_frame().get_motion_data()

# Save the Acceleration in accel_param
accel_param = np.array([0.127, 0.088, -9.287]).reshape(-1,3)

# Find the net Acceleration
accel_param_norm  = np.linalg.norm(accel_param)

# Acceleration if roll and pitch would have been zero
req_accel = np.array([0,0,-accel_param_norm]).reshape(-1,3)

# Get rotation vector that aligns the vector, effectively finding roll and pitch
rotated_vec = Rotation.align_vectors(req_accel,accel_param)

# Get euler angles from the rotation vectors
eul = rotated_vec[0].as_euler("xyz")

# Set Yaw as zero
eul[2] = 0

print(eul)
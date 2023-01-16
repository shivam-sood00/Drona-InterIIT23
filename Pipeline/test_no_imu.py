from vision_pipeline import VisionPipeline
#from kalman_filter import KalmanFilter
import pyrealsense2 as rs

import numpy as np
import time
import cv2
from cv2 import aruco

import csv

# used to record the time when we processed last frame
prev_frame_time = 0

# used to record the time at which we processed current frame
new_frame_time = 0

# open the file in the write mode
f = open('vision_data.csv', 'w')

# create the csv writer
writer = csv.writer(f)

# We need to set resolutions.
# so, convert them from float to integer.
frame_width = 1920
frame_height = 1080
   
size = (frame_width, frame_height)
   
# Below VideoWriter object will create
# a frame of above defined 
result = cv2.VideoWriter('Bhideo.avi', 
                         cv2.VideoWriter_fourcc(*'MJPG'),
                         30, size)


DEBUG = 1


if __name__ == '__main__':

    depth_res=(720, 1280)
    rgb_res=(1080, 1920)
    align_to="rgb"
    marker_size=3.6#13.8 #3.6
    marker_type=aruco.DICT_4X4_50
    required_marker_id = 1 #11 #1 #11
    calib_file_path="../calib_data/MultiMatrix.npz"

    pipeline = VisionPipeline(depth_res, rgb_res, align_to, marker_size, marker_type, required_marker_id, calib_file_path, debug=DEBUG)
    pipeline.init_realsense()
    pipeline.init_aruco_detector()

    # [x, y, z, x', y', z', roll, pitch, yaw, roll', pitch', yaw']
    init_X = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape((12, 1))
    init_P = np.zeros((12, 12))
    # Q = np.zeros((12, 12))
    Q = np.eye(12) * 0.2
    process_noise = np.zeros((12, 1))

    # Drone 
    mass = 0.1 # KG
    gravity = 9.8
    MOI = (0.1, 0.1, 0.1)

    control_input = np.array([1.0, 1.0, 1.0, 1.0]).reshape((4, 1))
    aruco_noise_bias = np.zeros((6, 1))
    aruco_noise_cov = np.zeros((6, 6))

    #kf = KalmanFilter(init_X.copy(), init_P.copy(), process_noise.copy(), Q.copy(), mass, gravity, MOI)

    last_time = time.time()

    try:
        while True:

            aligned_frames = pipeline.get_frames()    
            color_frame = pipeline.extract_rgb(aligned_frames)
            depth_frame = pipeline.extract_depth(aligned_frames)

            if not depth_frame or not color_frame:
                continue

            current_time = time.time()


            depth_img = pipeline.to_image(depth_frame)
            color_img = pipeline.to_image(color_frame)

            result.write(color_img)
            # time when we finish processing for this frame
            new_frame_time = time.time()
            fps = 1/(new_frame_time-prev_frame_time)
            prev_frame_time = new_frame_time
            fps = int(fps)
            print(fps)
            
            marker_corners = pipeline.detect_marker(color_img)
            
            if marker_corners is None:
                pass
            else:
                aruco_pose = pipeline.estimate_pose(marker_corners)
                aruco_pose[2][0] = aruco_pose[2][0] + 1.5 # Boards height
                dt = current_time - last_time
                last_time = current_time
                print(f"[{current_time}]: Aruco ESTIMATE: ", aruco_pose)

                # kf.apply_system_dynamics(control_input, dt)
                # print(f"[{current_time}]: System Dynamics ESTIMATE: ", kf.H @ kf.X)
                # kf.update_measurement(aruco_pose, aruco_noise_bias, aruco_noise_cov)
                # pose_estimate = (kf.H @ kf.X)

                # print(f"[{current_time}]: EKF ESTIMATE: ", pose_estimate)
                _intrinsics = rs.intrinsics()
                _intrinsics.width = 1920
                _intrinsics.height = 1080
                _intrinsics.ppx = 906.3662801147559
                _intrinsics.ppy = 561.2820445300187
                _intrinsics.fx = 1347.090250261588
                _intrinsics.fy = 1332.103727995465

                z_from_realsense = pipeline.depth_from_marker(depth_frame, marker_corners, kernel_size=3)
                # print(f"[{current_time}]: Z from REALSENSE [without TF]: ", z_from_realsense)
                mid_point = np.sum(marker_corners[0], 0) / 4.0
                mid_point = (mid_point + 0.5).astype(np.int32)
                
                point_from_rs = rs.rs2_deproject_pixel_to_point(_intrinsics, [mid_point[0], mid_point[1]], z_from_realsense)
                new_tf = pipeline.make_tf_matrix(rvec=pipeline.cam_rvec, tvec=np.array([0.0, 0.0, 0.0]))
                new_tf = np.linalg.pinv(new_tf)
                z_from_realsense = (new_tf @ np.array([point_from_rs[0] * 100.0, point_from_rs[1] * 100.0, point_from_rs[2] * 100.0, 1]).reshape((4, 1)))[2] / 100.0
                # print(f"[{current_time}]: Z from REALSENSE [After ROT]: ", z_from_realsense)
                dist_z_from_realsense = z_from_realsense[0] + 2.77
                print("aruco pose data:")
                print(aruco_pose)
                # print(f"[{current_time}]: Z from REALSENSE [After TF]: ", z_from_realsense)

                data  = [dt,current_time,dist_z_from_realsense]
                data.extend(aruco_pose.T[0].tolist())
                writer.writerow(data)

            key = cv2.waitKey(1)
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                result.release()
                break



    finally:
        pipeline.stop()
        f.close()
        

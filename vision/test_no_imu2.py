from vision_pipeline2 import VisionPipeline
import pyrealsense2 as rs
#from kalman_filter import KalmanFilter

import numpy as np
import time
import cv2
from cv2 import aruco
from scipy.spatial.transform import Rotation

import pandas as pd

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
moving_window_size = 10
moving_window_corners = []  
moving_window_pose = []

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
    marker_size=3.62 #13.8
    marker_type=aruco.DICT_4X4_50
    required_marker_id = 6
    calib_file_path="vision/calib_data/MultiMatrix.npz"

    calib_yaw_at_start = True
    imu_calib_data = [-0.03358463, 0.0135802, 0.0]

    pipeline = VisionPipeline(depth_res, rgb_res, align_to, marker_size, marker_type, required_marker_id, calib_file_path, debug=DEBUG,padding = 0, calib_yaw_at_start=calib_yaw_at_start, imu_calib_data=imu_calib_data)
    # pipeline = VisionPipeline(depth_res,rgb_res=(1080,1920),marker_size=3.6,required_marker_id=z,debug=1,padding = 0)

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

    pipeline.update_waypoint([-0.5, -0.0, 0.0])


    # aruco_x = []
    # aruco_y = []


    # normal_aruco = [[], []]
    # aruco_avg_corners = [[], []]
    # aruco_avg_pose = [[], []]

    pipeline.calib_yaw_at_start = True
    if pipeline.calib_yaw_at_start:
        max_iters = 100
        num_calib_frames = 0
        rvec_uncalib = []
        while True:
            aligned_frames = pipeline.get_frames()    
            color_frame = pipeline.extract_rgb(aligned_frames)
            depth_frame = pipeline.extract_depth(aligned_frames)

            if not depth_frame or not color_frame:
                continue
            
            color_img = pipeline.to_image(color_frame)
            marker_corners = pipeline.detect_marker(color_img)
            
            if marker_corners is None:
                pass
            elif type(marker_corners) is str:
                pass
            else:
                num_calib_frames += 1
                rvec_uncalib.append(pipeline.estimate_uncalib_pose(marker_corners))
                # if rvec_uncalib is None:
                #     rvec_uncalib = pipeline.estimate_uncalib_pose(marker_corners)
                # else:
                #     rvec_uncalib += pipeline.estimate_uncalib_pose(marker_corners)
                
                if(num_calib_frames >= max_iters):
                    print("yaw Caliberated ")
                    break 
        
        rvec_uncalib.sort()
        temp_ = Rotation.from_euler('xyz', np.array([0.0, 0.0, rvec_uncalib[int(len(rvec_uncalib) / 2.0)]])).as_rotvec()[2]
        pipeline.cam_rvec = np.array([0.0, 0.0, temp_+np.pi/2])
        # print(f"CAM RVEC: {Rotation.from_rotvec(pipeline.cam_rvec).as_euler('xyz')}")
    else:
        pipeline.cam_rvec = np.array([0.0, 0.0, 0.0])
    

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
            # print(color_img.shape)
            # color_img = cv2.resize(color_img, (1920, 1080), interpolation=4)

            # result.write(color_img)
            # time when we finish processing for this frame
            new_frame_time = time.time()
            fps = 1/(new_frame_time-prev_frame_time)
            prev_frame_time = new_frame_time
            fps = int(fps)
            print(fps)
            
            marker_corners = pipeline.detect_marker(color_img)
            
            if marker_corners is None:
                pass
            elif type(marker_corners) is str:
                pass
            else:


                if (len(moving_window_corners) >= 10):
                    moving_window_corners = moving_window_corners[1:]
                
                moving_window_corners.append(marker_corners)
                # print(moving_window_corners)
                avg_corners = (np.mean(moving_window_corners, 0) + 0.5).astype(int)


                ###
                aruco_pose = pipeline.estimate_pose(marker_corners)
                aruco_pose[0][0] = aruco_pose[0][0] * -1.0

                # normal_aruco[0].append(aruco_pose[0][0])
                # normal_aruco[1].append(aruco_pose[1][0])

                ###
                print("AVG CORNERS: ", avg_corners)
                aruco_pose_moving_corners = pipeline.estimate_pose(avg_corners)
                aruco_pose_moving_corners[0][0] = aruco_pose_moving_corners[0][0] * -1.0
                # print("moving corners pose", aruco_pose_moving_corners)
                # aruco_avg_corners[0].append(aruco_pose_moving_corners[0][0])
                # aruco_avg_corners[1].append(aruco_pose_moving_corners[1][0])
                
                if (len(moving_window_pose) >= 10):
                    moving_window_pose = moving_window_pose[1:]
                moving_window_pose.append(aruco_pose)

                avg_aruco_pose = np.mean(moving_window_pose, 0)

                # aruco_avg_pose[0].append(avg_aruco_pose[0][0])
                # aruco_avg_pose[1].append(avg_aruco_pose[1][0])


                dt = current_time - last_time
                last_time = current_time
                print(f"[{current_time}]: Aruco ESTIMATE: ")
                print(aruco_pose*100)

                # kf.apply_system_dynamics(control_input, dt)
                # print(f"[{current_time}]: System Dynamics ESTIMATE: ", kf.H @ kf.X)
                # kf.update_measurement(aruco_pose, aruco_noise_bias, aruco_noise_cov)
                # pose_estimate = (kf.H @ kf.X)

                # print(f"[{current_time}]: EKF ESTIMATE: ", pose_estimate)

                z_from_realsense = pipeline.depth_from_marker(depth_frame, marker_corners, kernel_size=3)
                _intrisics = rs.intrinsics()
                _intrisics.width = pipeline.rgb_res[1]
                _intrisics.height = pipeline.rgb_res[0]
                _intrisics.ppx = pipeline.cam_matrix[0][2]
                _intrisics.ppy = pipeline.cam_matrix[1][2]
                _intrisics.fx = pipeline.cam_matrix[0][0]
                _intrisics.fy = pipeline.cam_matrix[1][1]

                z_from_realsense = pipeline.depth_from_marker(depth_frame, marker_corners, kernel_size=3)
                mid_point = np.sum(marker_corners[0], 0) / 4.0
                mid_point = (mid_point + 0.5).astype(np.int32)

                pipeline.current_midpoint = mid_point.copy()

                print(f"[{current_time}]: Z from REALSENSE: ", z_from_realsense)
                new_tf = pipeline.make_tf_matrix(rvec=Rotation.from_euler('xyz', np.array(imu_calib_data)).as_rotvec(), tvec=np.array([0.0, 0.0, 0.0]))
                # if aruco_pose[0] >= x_threshold or aruco_pose[1] >= y_threshold:
                #     flag = 1
                #     cam_queue.append([flag])
                # else:
                #     flag = 0
                # print("appending")
                point_from_rs = rs.rs2_deproject_pixel_to_point(_intrisics, [mid_point[0], mid_point[1]],
                                                                z_from_realsense)
                # new_tf = pipeline.make_tf_matrix(rvec=pipeline.cam_rvec, tvec=np.array([0.0, 0.0, 0.0]))
                # new_tf = np.linalg.pinv(new_tf)
                z_from_realsense = (new_tf @ np.array(
                    [point_from_rs[0] * 100.0, point_from_rs[1] * 100.0, point_from_rs[2] * 100.0, 1]))[2] / 100.0
                
                z_from_realsense = -z_from_realsense + 2.858 #2.8189825700392364 + 0.04041281
                print(f"[{current_time}]: Z from REALSENSE: ", z_from_realsense)


                # aruco_x.append(aruco_pose.T[0][0])
                # aruco_y.append(aruco_pose.T[0][1])


                # data  = [dt,current_time,z_from_realsense]
                # data.extend(aruco_pose.T[0].tolist())
                # writer.writerow(data)

            key = cv2.waitKey(1)
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                result.release()
                break

    



    finally:
        
        # df = pd.DataFrame(list(zip(normal_aruco[0], normal_aruco[1], aruco_avg_corners[0], aruco_avg_corners[1], aruco_avg_pose[0], aruco_avg_pose[1])), columns=["nx", "ny", "cx", "cy", "px", "py"])
        # df.to_csv("temp_data__new.csv")
        
        # df = pd.DataFrame(list(zip(aruco_x, aruco_y)), columns=["x", "y"])
        # df.to_csv("Aruco_temp.csv")


        # print("Mean X: ", np.mean(aruco_x))
        # print("Std X: ", np.std(aruco_x))

        # print("Mean Y: ", np.mean(aruco_y))
        # print("Std Y: ", np.std(aruco_y))

        pipeline.stop()
        f.close()
        

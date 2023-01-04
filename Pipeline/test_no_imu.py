from vision_pipeline import VisionPipeline
from kalman_filter import KalmanFilter

import numpy as np
import time
import cv2
from cv2 import aruco

DEBUG = 1


if __name__ == '__main__':

    depth_res=(720, 1280)
    rgb_res=(720, 1280)
    align_to="rgb"
    marker_size=3.75
    marker_type=aruco.DICT_4X4_50
    required_marker_id=0
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

    kf = KalmanFilter(init_X.copy(), init_P.copy(), process_noise.copy(), Q.copy(), mass, gravity, MOI)

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
            
            marker_corners = pipeline.detect_marker(color_img)
            
            if marker_corners is None:
                pass
            else:
                aruco_pose = pipeline.estimate_pose(marker_corners)
                dt = current_time - last_time
                last_time = current_time
                print(f"[{current_time}]: Aruco ESTIMATE: ", aruco_pose)

                kf.apply_system_dynamics(control_input, dt)
                print(f"[{current_time}]: System Dynamics ESTIMATE: ", kf.H @ kf.X)
                kf.update_measurement(aruco_pose, aruco_noise_bias, aruco_noise_cov)
                pose_estimate = (kf.H @ kf.X)

                print(f"[{current_time}]: EKF ESTIMATE: ", pose_estimate)

                z_from_realsense = pipeline.depth_from_marker(depth_frame, marker_corners, kernel_size=3)
                print(f"[{current_time}]: Z from REALSENSE: ", z_from_realsense)

            key = cv2.waitKey(1)
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break



    finally:
        pipeline.stop()
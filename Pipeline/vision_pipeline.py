import pyrealsense2 as rs
import numpy as np
import cv2
from cv2 import aruco
import time
import os
from scipy.spatial.transform import Rotation

class VisionPipeline():

    def __init__(self,
                 depth_res=(720, 1280),
                 rgb_res=(720, 1280),
                 align_to="rgb",
                 marker_size=3.75,
                 marker_type=aruco.DICT_4X4_50,
                 required_marker_id=0,
                 calib_file_path="../calib_data/MultiMatrix.npz",
                 debug=0) -> None:


        self.depth_res = depth_res
        self.rgb_res = rgb_res
        self.align_to = align_to


        self.calib_file_path = calib_file_path

        self.marker_size = marker_size
        self.marker_type = marker_type
        self.marker_dict = aruco.Dictionary_get(self.marker_type)

        self.required_marker_id = required_marker_id
        
        self.DEBUG = debug
        pass


    def init_realsense(self):
        self.pipeline = rs.pipeline()

        config = rs.config()
        config.enable_stream(rs.stream.depth, self.depth_res[1], self.depth_res[0], rs.format.z16, 30)
        config.enable_stream(rs.stream.color, self.rgb_res[1], self.rgb_res[0], rs.format.bgr8, 30)
        
        profile = self.pipeline.start(config)

        self.depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()


        if self.align_to.lower() == "rgb":
            self.align_frames = rs.align(rs.stream.color)
        elif self.align_to.lower() == "depth":
            self.align_frames = rs.align(rs.stream.depth)
        else:
            raise NotImplementedError(f"Align to {self.align_to} not implemented!")


    
    def init_aruco_detector(self):
        
        if not os.path.exists(self.calib_file_path):
            raise FileNotFoundError(f"File {self.calib_file_path} not found!")

        calib_data = np.load(self.calib_file_path)
        self.cam_matrix = calib_data["camMatrix"]
        self.dist_coef = calib_data["distCoef"]
        self.cam_rvec = np.array([0, 0, 0]) #calib_data["rVector"]
        self.cam_tvec = np.array([0, 0, 0]) #calib_data["tVector"]


        self.cam_tf = np.linalg.pinv(self.make_tf_matrix(self.cam_rvec, self.cam_tvec))


        self.param_markers = aruco.DetectorParameters_create()
        

    def stop(self):
        self.pipeline.stop()


    def get_frames(self):
        self.frames = self.pipeline.wait_for_frames()
        self.aligned_frames = self.align_frames.process(self.frames)

        return self.aligned_frames

    
    def extract_depth(self, frame):
        return frame.get_depth_frame()

    
    def extract_rgb(self, frame):
        return frame.get_color_frame()


    def to_image(self, frame):
        return np.asarray(frame.get_data())

    
    def detect_marker(self, frame):
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        marker_corners, marker_IDs, reject = aruco.detectMarkers(
            gray_frame, self.marker_dict, parameters=self.param_markers
        )

        if self.DEBUG:
            frame = self.plot_markers(frame, marker_corners, marker_IDs)
            self.show_frame(frame)

        if marker_IDs is None:
            if self.DEBUG:
                print("NO marker detected")
            return None

        for i, id_ in enumerate(marker_IDs):
            
            if id_ == self.required_marker_id:
                return marker_corners[i].astype(np.int32)

        return None



    def plot_markers(self, frame, marker_corners, marker_ids):
        frame = frame.copy()
        for i, corners in enumerate(marker_corners):
            if marker_ids[i] == self.required_marker_id:
                frame = cv2.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv2.LINE_AA)
            else:
                frame = cv2.polylines(frame, [corners.astype(np.int32)], True, (0, 0, 255), 4, cv2.LINE_AA)

        return frame


    def show_frame(self, frame, window_name="Frame"):
        cv2.imshow(window_name, frame)



    def estimate_pose(self, marker_corners):
        print(marker_corners)
        print(self.cam_matrix)
        print(self.dist_coef)
        rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
                [marker_corners.astype(np.float32)], self.marker_size, self.cam_matrix, self.dist_coef
            )

        tf = self.make_tf_matrix(rVec[0, 0, :], tVec[0, 0, :])        
        tf = self.cam_tf @ tf
        return self.tf_to_outformat(tf)



    def make_tf_matrix(self, rvec, tvec):
        rot_mat = Rotation.from_rotvec(rvec).as_matrix()
        tf = np.eye(4)
        tf[:3, :3] = rot_mat
        tf[:3, 3] = tvec
        return tf

    
    def tf_to_outformat(self, tf):
        out_vec = np.zeros((6, 1))
        out_vec[3:, 0] = Rotation.from_matrix(tf[:3, :3]).as_euler('xyz')
        out_vec[:3, 0] = tf[:3, 3]
        return out_vec


    def outformat_to_tf(self, input):
        tf = np.eye(4)
        tf[:3, :3] = Rotation.from_euler('xyz', input[3:, 0]).as_matrix()
        tf[:3, 3] = input[:3, 0]
        return tf


    def depth_from_marker(self, depth_frame, marker_corners, kernel_size=1):
        mid_point = np.sum(marker_corners[0], 0) / 4.0
        mid_point = (mid_point + 0.5).astype(np.int32)

        depths = []
        for j in range(-int(kernel_size / 2.0), int(kernel_size / 2.0) + 1):
            for k in range(-int(kernel_size / 2.0), int(kernel_size / 2.0) + 1):
                try:
                    depth = depth_frame.get_distance(mid_point[0] + j, mid_point[1] + k)
                    depths.append(depth)
                except:
                    pass

        depths.sort()
        num_values = len(depths)
        if num_values % 2 == 0:
            return (depths[int(num_values / 2.0)] + depths[int(num_values / 2.0) - 1]) / 2.0
        else:
            return depths[int(num_values / 2.0) - 1]
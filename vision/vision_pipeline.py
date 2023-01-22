import pyrealsense2 as rs
import numpy as np
import cv2
from cv2 import aruco
import time
import os
from scipy.spatial.transform import Rotation
import csv

class VisionPipeline():
    """
        This is a class for initializing Intel Pyrealsense2 D435i camera and also finding 
        depth and rgb frames along with detection of aruco marker

        Attributes:
            depth_res: defines the depth resolution. Default is (720, 1080).
            rgb_res: defines the rgb resolution. Default is (720, 1080).
            align_to: defines frame to which alignment must be done. Default is "rgb".
            marker_size: defines the marker size of the aruco tag. Default is 3.75.
            marker_type: defines the aruco marker type. Default is DICT_4X4_50.
            marker_dict: gets the required aruco dictionary.
            require_marker_id: defines the marker id of the aruco tag placed on the drone.
            calib_file_path: defines the path for camera calibration.
            DEBUG: a flag to enable prints in code for debugging. Default is 0.
        
    """
    def __init__(self,
                 depth_res=(720, 1280),
                 rgb_res=(720, 1280),
                 align_to="rgb",
                 marker_size=3.75,
                 marker_type=aruco.DICT_4X4_50,
                 required_marker_id=0,
                 calib_file_path="../calib_data/MultiMatrix.npz",
                 debug=0,
                 padding=50) -> None:


        self.depth_res = depth_res
        self.rgb_res = rgb_res
        self.align_to = align_to


        self.calib_file_path = calib_file_path

        self.marker_size = marker_size
        self.marker_type = marker_type
        self.marker_dict = aruco.getPredefinedDictionary(self.marker_type)

        self.required_marker_id = required_marker_id
        
        self.DEBUG = debug
        self.padding = padding
        pass


    def init_realsense(self):
        """
        Initializes realsense, Enables both depth and rgb stream

        Parameter:
            None         

        Returns:
            None
        """   
        self.pipeline = rs.pipeline()

        config = rs.config()
        config.enable_stream(rs.stream.depth, self.depth_res[1], self.depth_res[0], rs.format.z16, 30)
        config.enable_stream(rs.stream.color, self.rgb_res[1], self.rgb_res[0], rs.format.bgr8, 30)
        
        profile = self.pipeline.start(config)

        colorSensor = profile.get_device().query_sensors()[1];

        # rs.option.enable_auto_exposure
        rs.option.enable_motion_correction
        # print(colorSensor.get_info(rs.camera_info.name))

        colorSensor.set_option(rs.option.enable_auto_exposure, 0)
        colorSensor.set_option(rs.option.enable_auto_white_balance, 0)

        colorSensor.set_option(rs.option.sharpness, 100)
        colorSensor.set_option(rs.option.contrast, 50)
        # colorSensor.set_option(rs.option.gamma, 0)
        colorSensor.set_option(rs.option.brightness, 30)

        colorSensor.set_option(rs.option.exposure, 100)
        colorSensor.set_option(rs.option.gain, 300)

        self.depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()


        if self.align_to.lower() == "rgb":
            self.align_frames = rs.align(rs.stream.color)
        elif self.align_to.lower() == "depth":
            self.align_frames = rs.align(rs.stream.depth)
        else:
            raise NotImplementedError(f"Align to {self.align_to} not implemented!")


    
    def init_aruco_detector(self):
        """
        Calibrates the camera with Intrinsic Parameters and Extrinsic Parameters and generates parameters for marker
        
        Parameters:
            None
        
        Returns:
            None
        """
        
        # if not os.path.exists(self.calib_file_path):
        #     raise FileNotFoundError(f"File {self.calib_file_path} not found!")

        # calib_data = np.load(self.calib_file_path)
        fx = 640.381164550781
        cx = 631.432983398438
        fy = 639.533020019531
        cy = 409.294647216797
        self.cam_matrix = np.array([[1347.090250261588, 0, 906.3662801147559],[0, 1332.103727995465, 561.2820445300187],[0,0,1]]) #calib_data["camMatrix"]
        self.dist_coef = np.array([0.1269819023042869, -0.4583739190940396, 0.002002457353149274, -0.01606097632795915, 0.3598527092759298]) #np.zeros((5, 1))  #calib_data["distCoef"]
        """dist_coef = np.zeros((5, 1))"""

        self.cam_rvec = np.array([0.10357627, -2.8488926,  -0.55131484])
        self.cam_tvec = np.array([46.22983901,   1.60285046, 278.0799618])
        self.cam_tf = np.linalg.pinv(self.make_tf_matrix(self.cam_rvec, self.cam_tvec))


        self.param_markers = aruco.DetectorParameters_create()
        

    def stop(self):
        """
        Stops pipeline

        Parameters:
            None
        
        Returns:
            None
        """
        self.pipeline.stop()


    def get_frames(self):
        """
        Aligns depth and rgb frames

        Parameters:
            None
        
        Returns:
            aligned frames
            
        """
        self.frames = self.pipeline.wait_for_frames()
        self.aligned_frames = self.align_frames.process(self.frames)

        return self.aligned_frames

    
    def extract_depth(self, frame):
        """
        Extracts depth frame

        Parameters:
            frame
        
        Returns:
            depth frame
        """
        return frame.get_depth_frame()

    
    def extract_rgb(self, frame):
        """
        Extracts rgb frame

        Parameters:
            frame
        
        Returns:
            colour frame
        """
        return frame.get_color_frame()


    def to_image(self, frame):
        """
        converts pyrealsense2.frame to np.array

        Parameters:
            frame: input frame of type pyrealsense2.frame
        
        Returns:
            numpy array
        """
        return np.asarray(frame.get_data())

    
    def detect_marker(self, frame):
        """
        Detection of aruco markers returning various aruco parameters like id and corner

        Parameters:
            frame: input frame of type pyrealsense2.frame
        
        Returns:
            corner list of aruco tag
        """
        # frame = cv2.medianBlur(frame, 3)
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        marker_corners, marker_IDs, reject = aruco.detectMarkers(
            gray_frame, self.marker_dict, parameters=self.param_markers
        )

        if self.DEBUG:
            frame = self.plot_markers(frame, marker_corners, marker_IDs)
            frame = self.plot_rej_markers(frame, reject)
            cv2.imwrite(f"frames/{time.time()}.jpg", frame)
            self.show_frame(frame)

        if marker_IDs is None:
            if self.DEBUG:
                print("NO marker detected")
            return None

        for i, id_ in enumerate(marker_IDs):
            
            if id_ == self.required_marker_id:
                # print(marker_corners[i])
                mid_point = np.sum(marker_corners[i][0], 0) / 4.0
                # print(mid_point)
                if (mid_point[0] >= self.rgb_res[1] - self.padding) or (mid_point[0] <= self.padding) or (mid_point[1] >= self.rgb_res[0] - self.padding) or (mid_point[1] <= self.padding):
                    return "None"

                return marker_corners[i].astype(np.int32)

        return None



    def plot_markers(self, frame, marker_corners, marker_ids):
        """
        Draws lines around the detected frames

        Parameters:
            frame
            marker_corners: corners of the detected aruco tag
            marker_ids: ids of the detected markers
        
        Returns:
            frame with plotted markers
        """
        frame = frame.copy()
        for i, corners in enumerate(marker_corners):
            if marker_ids[i] == self.required_marker_id:
                frame = cv2.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv2.LINE_AA)

            else:
                frame = cv2.polylines(frame, [corners.astype(np.int32)], True, (0, 0, 255), 4, cv2.LINE_AA)

        return frame

    def plot_rej_markers(self, frame, marker_corners):
        """
        Draws lines around the detected frames

        Parameters:
            frame
            marker_corners: corners of the detected aruco tag            
        
        Returns:
            frame with plotted rejected markers
        """
        frame = frame.copy()
        for i, corners in enumerate(marker_corners):
            frame = cv2.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 0), 4, cv2.LINE_AA)

        return frame


    def show_frame(self, frame, window_name="Frame"):
        """
        Displays frame

        Parameters:
            frame:
            window_name:
        
        Returns:
            None
        """
        cv2.imshow(window_name, frame)



    def estimate_pose(self, marker_corners, frame_of_ref="camera"):
        """
        Estimates the pose of the transform matrix using large aruco tag

        Parameters:
            list of marker corners
            frame of reference. Default value is "camera"
        
        Returns:
            6-DOF pose estimate of aruco marker in world frame
            
        """
        #print(marker_corners)
        #print(self.cam_matrix)
        #print(self.dist_coef)
        
        rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
                [marker_corners.astype(np.float32)], self.marker_size, self.cam_matrix, self.dist_coef
            )
        # print("RVec:",rVec,"Tvec:",tVec)
        tf = self.make_tf_matrix(np.array([0, 0, 0]), tVec[0, 0, :])
        if frame_of_ref == "camera":
            
            correction_tf = self.make_tf_matrix(rvec=Rotation.from_euler('xyz', np.array([-0.0073827, 0.00310284, 0])).as_rotvec(), tvec=np.array([0.0, 0.0, 0.0]))
            tf = correction_tf @ tf
            
            translation_tf = self.make_tf_matrix(rvec=Rotation.from_euler('xyz', np.array([0, 0, 0])).as_rotvec(), tvec=self.cam_tvec)
            translation_tf = np.linalg.pinv(translation_tf)
            tf = translation_tf @ tf
            pass
        else:        
            tf = self.cam_tf @ tf
        return self.tf_to_outformat(tf) / 100.0


    def make_tf_matrix(self, rvec, tvec):
        """
        Creates the transormation matrix using the rotation and translational vectors

        Parameters:
            rvec: contains rotational vectors
            tvec: contains trsnslational vectors
        
        Returns:
            Transformation matrix for the camera
        """
        rot_mat = Rotation.from_rotvec(rvec).as_matrix()
        tf = np.eye(4)
        tf[:3, :3] = rot_mat
        tf[:3, 3] = tvec
        return tf

    
    def tf_to_outformat(self, tf):
        """
        converts the transformation matrix to a list

        Parameters:
            tf: Transformation matrix
        
        Returns:
            out_vec: conversion of the transformation matrix into list
        """
        out_vec = np.zeros((6, 1))
        out_vec[3:, 0] = Rotation.from_matrix(tf[:3, :3]).as_euler('xyz')
        out_vec[:3, 0] = tf[:3, 3]
        return out_vec


    def outformat_to_tf(self, input):
        """
        converts the list to the transformation matrix 

        Parameters:
            out_vec: conversion of the transformation matrix into list            
        
        Returns:            
            tf: Transformation matrix
        """
        tf = np.eye(4)
        tf[:3, :3] = Rotation.from_euler('xyz', input[3:, 0]).as_matrix()
        tf[:3, 3] = input[:3, 0]
        return tf


    def depth_from_marker(self, depth_frame, marker_corners, kernel_size=1):
        """
        Finds depth from the estimated pose obtained from the aruco tag

        Parameters:
            depth_frame:
            marker_corners: contains the list of aruco marker corners
            kernel_size: contains the size of the kernel
        Returns:
            depths[]: depth of the point filtered using median filter
        """
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

    def cam_init(self):
        """
        Initializes camera and starts aruco detection
        Parameters:
            None
        Returns:
            None
        """
        self.init_realsense()
        self.init_aruco_detector()
        
    def cam_process(self, cam_queue):
        counter = 0
        while True:
            # print("cam##############################################################loop")
            flag = 0
            x_threshold = 0
            y_threshold = 0
            aligned_frames = self.get_frames()    
            color_frame = self.extract_rgb(aligned_frames)
            depth_frame = self.extract_depth(aligned_frames)

            if not depth_frame or not color_frame:
                continue

            current_time = time.time()

            color_img = self.to_image(color_frame)

            # time when we finish processing for this frame
            # new_frame_time = time.time()
            # fps = 1/(new_frame_time-prev_frame_time)
            # prev_frame_time = new_frame_time
            # fps = int(fps)
            # print(fps)
            
            marker_corners = self.detect_marker(color_img)
            
            if marker_corners is None:
                counter += 1
                if counter >= 15:
                    flag = 2
                    cam_queue.append([flag])
                    counter = 0
                # print("no aruco")
                pass
            elif type(marker_corners) == type("None"):
                    flag = 1
                    cam_queue.append([flag])
            else:
                # print("detected")
                counter = 0
                aruco_pose = self.estimate_pose(marker_corners)
                #dt = current_time - last_time
                #last_time = current_time
                #print(f"[{current_time}]: Aruco ESTIMATE: ", aruco_pose)

                # kf.apply_system_dynamics(control_input, dt)
                # print(f"[{current_time}]: System Dynamics ESTIMATE: ", kf.H @ kf.X)
                # kf.update_measurement(aruco_pose, aruco_noise_bias, aruco_noise_cov)
                # pose_estimate = (kf.H @ kf.X)

                # print(f"[{current_time}]: EKF ESTIMATE: ", pose_estimate)
                #####################################################################
                _intrisics = rs.intrinsics()
                _intrisics.width = self.rgb_res[1]
                _intrisics.height = self.rgb_res[0]
                _intrisics.ppx = self.cam_matrix[0][2]
                _intrisics.ppy = self.cam_matrix[1][2]
                _intrisics.fx = self.cam_matrix[0][0]
                _intrisics.fy = self.cam_matrix[1][1]

                z_from_realsense = self.depth_from_marker(depth_frame, marker_corners, kernel_size=3)
                mid_point = np.sum(marker_corners[0], 0) / 4.0
                mid_point = (mid_point + 0.5).astype(np.int32)
                
                #print(f"[{current_time}]: Z from REALSENSE: ", z_from_realsense)
                # if aruco_pose[0] >= x_threshold or aruco_pose[1] >= y_threshold:
                #     flag = 1
                #     cam_queue.append([flag])
                # else:
                #     flag = 0
                    # print("appending")
                new_tf = self.make_tf_matrix(rvec=Rotation.from_euler('xyz', np.array([-0.0073827, 0.00310284, 0])).as_rotvec(), tvec=np.array([0.0, 0.0, 0.0]))
                
                point_from_rs = rs.rs2_deproject_pixel_to_point(_intrisics, [mid_point[0], mid_point[1]], z_from_realsense)
                # new_tf = self.make_tf_matrix(rvec=self.cam_rvec, tvec=np.array([0.0, 0.0, 0.0]))
                # new_tf = np.linalg.pinv(new_tf)
                z_from_realsense = (new_tf @ np.array([point_from_rs[0] * 100.0, point_from_rs[1] * 100.0, point_from_rs[2] * 100.0, 1]))[2] / 100.0
                z_from_realsense = -z_from_realsense + 2.858
                ##############################################################################
                # print("Z: ", z_from_realsense)
                
                aruco_pose[0] = -aruco_pose[0]
                
                flag=0
                cam_queue.append([current_time, aruco_pose, z_from_realsense])

                # data  = [dt,current_time,z_from_realsense]
                # data.extend(aruco_pose.T[0].tolist())
            
            cv2.imshow("RGB Image",color_img)
                

            key = cv2.waitKey(1)
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                
                break
    
    
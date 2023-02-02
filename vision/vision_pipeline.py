import pyrealsense2 as rs
import numpy as np
import cv2
from cv2 import aruco
import time
from scipy.spatial.transform import Rotation
import math
import yaml

import wandb

class VisionPipeline():
    """
        This is a class for initializing Intel Pyrealsense2 D435i camera and also finding 
        depth and rgb frames along with detection of aruco marker.
       
        Attributes:
            depth_res: defines the depth resolution. Default is (720, 1080).
            rgb_res: defines the rgb resolution. Default is (720, 1080).
            align_to: defines frame to which alignment must be done. Default is "rgb".
            marker_size: defines the marker size of the aruco tag. Default is 3.75.
            marker_type: defines the aruco marker type. Default is DICT_4X4_50.
            marker_dict: gets the required aruco dictionary.
            require_marker_id: defines the marker id of the aruco tag placed on the drone.
            calib_file_path: defines the path for camera calibration.
            DEBUG: a flag to enable prints in code for debugging. Default is False.
        
    """
    def __init__(self,
                 depth_res=(720, 1280),
                 rgb_res=(720, 1280),
                 align_to="rgb",
                 marker_size=3.75,
                 marker_type=aruco.DICT_4X4_50,
                 required_marker_id=1,
                 debug=0,
                 padding=50,
                 config_file="./vision/config.yaml",
                 fps_moving_window_size=10) -> None:


        self.depth_res = depth_res
        self.rgb_res = rgb_res
        self.align_to = align_to
        self.marker_size = marker_size
        self.marker_type = marker_type
        self.marker_dict = aruco.getPredefinedDictionary(self.marker_type)

        self.required_marker_id = required_marker_id

        with open(config_file, 'r') as f:
            self.camera_config = yaml.load(f)
        
        self.camera_id = self.camera_config['active_camera']
        self.camera_intrinsic = self.camera_config['camera'][self.camera_id]['intrinsics']
        self.camera_extrinsic = self.camera_config['camera'][self.camera_id]['extrinsics']
        
        self.DEBUG = debug
        self.padding = padding
        self.fps_moving_window_size = fps_moving_window_size
        self.fps_moving_window = []

        self.do_tracking = self.camera_config['tracking']['enable']
        self.tracking_area_th = self.camera_config['tracking']['area_th']
        self.tracking_point_th = self.camera_config['tracking']['centroid_th']
        
        self.markerCornerEstimates = {self.required_marker_id[0]:None,self.required_marker_id[1]:None}
        self.estimated_pose_all = {self.required_marker_id[0]:None,self.required_marker_id[1]:None}
        self.current_waypoint = {self.required_marker_id[0]:None,self.required_marker_id[1]:None}
        self.current_midpoint = {self.required_marker_id[0]:None,self.required_marker_id[1]:None}

        self.calib_yaw_at_start = self.camera_extrinsic['calibrate_yaw']
        self.imu_calib_data = self.camera_extrinsic['imu_correction']

        self.cam_init()

        if self.camera_config['wandb']['use_wandb']:
            self.wandb_init()

        self.last_time = None
        self.current_time = None
        self.counter = 0
        
        self.avg_fps = None
        self.cam_rvec = np.array([0.0, 0.0, 0.0])
        self.cam_rvecs = {self.required_marker_id[0]:np.array([0.0, 0.0, 0.0]), self.required_marker_id[1]:np.array([0.0, 0.0, 0.0])}
        self.raw_calib_yaws = {self.required_marker_id[0]:0.0, self.required_marker_id[1]:0.0}

        self.rp_correction = Rotation.from_euler('xyz', np.array(self.imu_calib_data)).as_matrix()

        if self.calib_yaw_at_start:
            self.calibrate_yaw()
        else:
            self.cam_rvec = np.array(self.camera_extrinsic['default_yaw']) #np.array([0.0, 0.0, 0.0])
            self.cam_rvecs = {self.required_marker_id[0]:np.array(self.camera_extrinsic['default_yaw']),self.required_marker_id[1]:np.array(self.camera_extrinsic['default_yaw'])} #np.array([0.0, 0.0, 0.0])
            self.raw_calib_yaws = {self.required_marker_id[0]:np.array(self.camera_extrinsic['default_yaw'])[2],self.required_marker_id[1]:np.array(self.camera_extrinsic['default_yaw'])[2]} # 0.0
        
        self.yaw_correction = Rotation.from_rotvec(self.cam_rvec).as_matrix()
        self.yaw_correction = np.linalg.pinv(self.yaw_correction)
        self.rpy_correction = self.yaw_correction @ self.rp_correction 



    def wandb_init(self):
        wandb.init(project="inter-iit", config=self.camera_config)
        pass        


    
    def init_realsense(self):
        """
        Initializes Realsense by enabling both depth and RGB stream and sets up parameters such as sharpness, contrast, exposure etc.
        
        Parameter:
            None         
        
        Returns:
            None
        """

        # Start realsense pipeline
        self.pipeline = rs.pipeline()

        config = rs.config()
        config.enable_stream(rs.stream.depth, self.depth_res[1], self.depth_res[0], rs.format.z16, 30)
        config.enable_stream(rs.stream.color, self.rgb_res[1], self.rgb_res[0], rs.format.bgr8, 30)
        
        profile = self.pipeline.start(config)

        colorSensor = profile.get_device().query_sensors()[1]

        # Set camera parameters

        # rs.option.enable_auto_exposure
        rs.option.enable_motion_correction

        colorSensor.set_option(rs.option.enable_auto_exposure, self.camera_config['camera_params']['enable_auto_exposure'])
        colorSensor.set_option(rs.option.enable_auto_white_balance, self.camera_config['camera_params']['enable_auto_white_balance'])

        colorSensor.set_option(rs.option.sharpness, self.camera_config['camera_params']['sharpness'])
        colorSensor.set_option(rs.option.contrast, self.camera_config['camera_params']['contrast'])
        # colorSensor.set_option(rs.option.gamma, 0)
        colorSensor.set_option(rs.option.brightness, self.camera_config['camera_params']['brightness'])

        colorSensor.set_option(rs.option.exposure, self.camera_config['camera_params']['exposure'])
        # colorSensor.set_option(rs.option.gain, 300)

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
        Initializes camera Intrinsic and Extrinsic Parameters and generates Aruco detector parameters.
        
        Parameters:
            None
        
        Returns:
            None
        """ 

        self.cam_matrix = np.array(self.camera_intrinsic['cam_matrix']) #calib_data["camMatrix"]
        self.dist_coef = np.array(self.camera_intrinsic['dist_coeff']) #np.zeros((5, 1))  #calib_data["distCoef"]
        self.cam_rvec = np.array([0.0, 0.0, 0.0])
        self.cam_tvec = np.array(self.camera_extrinsic['global_origin'])
        self.cam_tf = np.linalg.pinv(self.make_tf_matrix(self.cam_rvec, self.cam_tvec))


        self.param_markers = aruco.DetectorParameters_create()
        

    def stop(self):
        """
        Stops the pipeline.
        
        Parameters:
            None
        
        Returns:
            None
        """
        self.pipeline.stop()


    def get_frames(self):
        """
        Returns the aligned depth and rgb frames for proper depth detection.
        
        Parameters:
            None
        
        Returns:
            aligned frames: Returns frame with depth and RGB frame aligned.
            
        """
        self.frames = self.pipeline.wait_for_frames()
        self.aligned_frames = self.align_frames.process(self.frames)

        return self.aligned_frames

    
    def extract_depth(self, frame):
        """
        Returns depth frame from Realsense Depth Camera.
        
        Parameters:
            frame
        
        Returns:
            depth frame
        """
        return frame.get_depth_frame()

    
    def extract_rgb(self, frame):
        """
        Extracts RGB frame from Realsense RGB Camera.
        
        Parameters:
            frame
        
        Returns:
            frame.get_color_frame(): colour frame
        """
        return frame.get_color_frame()


    def to_image(self, frame):
        """
        Converts pyrealsense2.frame to np.array.
        
        Parameters:
            frame: input frame of type pyrealsense2.frame
        
        Returns:
            np.asarray(frame.get_data())
        """
        return np.asarray(frame.get_data())
    
    
    def find_area(self, corners):
        """
        Utility function to find area of the quadrilateral using corner data.
        
        Parameters:
            corners: co-ordinate of the corners whose area needs to be calculated
        
        Returns:
            area:  returns area of the quadrilateral, specified by its corners
        """  
        corners = corners[0]
        area = (corners[0][0] * corners[1][1] + corners[1][0] * corners[2][1] + corners[2][0] * corners[3][1] + corners[3][0] * corners[0][1])
        area = area - (corners[1][0] * corners[0][1] + corners[2][0] * corners[1][1] + corners[3][0] * corners[2][1] + corners[0][0] * corners[3][1])
        return area

    
    def detect_marker(self, frame):
        """
        Detection of ArUco markers returning its corners.
        
        Parameters:
            frame: input frame of type numpy.array
        
        Returns:
            Case 1:
                None : Returns if no marker detected
            Case 2:
                marker_corners : Returns Corners of detected Aruco Marker
            Case 3:
                "None" : Returns a string, if goes out of region of interest
        """
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        marker_corners, marker_IDs, reject = aruco.detectMarkers(
            gray_frame, self.marker_dict, parameters=self.param_markers
        )

        if self.DEBUG:
            frame = self.plot_markers(frame, marker_corners, marker_IDs)
            rgb_frame = self.plot_markers(frame.copy(), marker_corners, marker_IDs)

        if marker_IDs is None:
            
            if (self.do_tracking == True):
                
                if (reject is None):
                    if self.DEBUG:
                        frame = self.plot_rej_markers(frame, reject)
                        rgb_frame = self.plot_rej_markers(rgb_frame, reject)
                        self.show_frame(frame, rgb_frame)
                        print("NO marker detected")
                    return self.markerCornerEstimates
                
                else:
                    
                    for (key, last_marker) in self.markerCornerEstimates.items():
                        if last_marker is None:
                            frame = self.plot_rej_markers(frame, reject)
                            rgb_frame = self.plot_rej_markers(rgb_frame, reject)
                            self.show_frame(frame, rgb_frame)
                            print("NO marker detected")
                            return self.markerCornerEstimates

                        last_area = self.find_area(last_marker)
                        last_center = np.mean(last_marker[0], 0) / 4.0
                        
                        for i, reject_corners in enumerate(reject):
                            new_center = np.mean(reject_corners[0], 0) / 4.0
                            new_area = self.find_area(reject_corners)
                            
                            if((abs(new_area - last_area) <= self.tracking_area_th) and (math.sqrt(np.sum((new_center - last_center) ** 2)) <= self.tracking_point_th)):
                                self.markerCornerEstimates[key] = reject_corners.copy().astype(np.int32)
                                
                                if self.DEBUG:
                                    frame = cv2.polylines(frame, [reject_corners.astype(np.int32)], True, (255, 0, 0), 4, cv2.LINE_AA)
                                    rgb_frame = cv2.polylines(rgb_frame, [reject_corners.astype(np.int32)], True, (255, 0, 0), 4, cv2.LINE_AA)
                                    self.show_frame(frame, rgb_frame)
                                    
                                # self.reject_corners.astype(np.int32)
                            
                            else:
                                
                                if self.DEBUG:
                                    print(f"NO TRACKING {key}: ", new_area, new_center)
                                    print(f"LAST {key}: ", last_area, last_center)
                                    
                    if self.DEBUG:
                        frame = self.plot_rej_markers(frame, reject)
                        rgb_frame = self.plot_rej_markers(rgb_frame, reject)
                        print("NO marker detected!")
                        self.show_frame(frame, rgb_frame)
                    return self.markerCornerEstimates
            
            else:
                if (reject is None):
                    if self.DEBUG:
                        frame = self.plot_rej_markers(frame, reject)
                        rgb_frame = self.plot_rej_markers(rgb_frame, reject)
                        self.show_frame(frame, rgb_frame)
                        print("NO marker detected")
                    return self.markerCornerEstimates
                    
        else:
            if self.DEBUG:
                frame = self.plot_rej_markers(frame, reject)
                rgb_frame = self.plot_rej_markers(rgb_frame, reject)
                self.show_frame(frame, rgb_frame)            
                        
            for i, id_ in enumerate(marker_IDs):
                id_ = id_[0]
                if id_ in self.required_marker_id:
                    mid_point = np.sum(marker_corners[i][0], 0) / 4.0
                    if (mid_point[0] >= self.rgb_res[1] - self.padding) or (mid_point[0] <= self.padding) or (mid_point[1] >= self.rgb_res[0] - self.padding) or (mid_point[1] <= self.padding):
                        # return "None"
                        self.markerCornerEstimates[id_] = "Landing"
                    else:
                        self.last_detected_marker = marker_corners[i].copy()
                        # return marker_corners[i].astype(np.int32)
                        # print(self.markerCornerEstimates,id_,marker_corners)
                        self.markerCornerEstimates[id_] = marker_corners[i].astype(np.int32)
            
        return self.markerCornerEstimates



    def plot_markers(self, frame, marker_corners, marker_ids):
        """
        Draws lines around the detected frames.
        
        Parameters:
            frame: RGB frame
            marker_corners: corners of the detected aruco tag
            marker_ids: ids of the detected markers
        
        Returns:
            frame with plotted markers
        """
        
        frame = frame.copy()
        for i, corners in enumerate(marker_corners):
            if marker_ids[i] in self.required_marker_id:
                frame = cv2.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 0), 4, cv2.LINE_AA)

            else:
                frame = cv2.polylines(frame, [corners.astype(np.int32)], True, (0, 0, 255), 4, cv2.LINE_AA)

        return frame

    def plot_rej_markers(self, frame, marker_corners):
        """
        Draws lines around the detected frames.
        
        Parameters:
            frame: RGB frame
            marker_corners: corners of the detected aruco tag            
        
        Returns:
            frame: frame with plotted rejected markers
        """
        frame = frame.copy()
        for i, corners in enumerate(marker_corners):
            frame = cv2.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv2.LINE_AA)

        return frame


    def update_waypoint(self, waypoint, id_):
        """
        Updates the waypoint that is plotted on the image frame to be displayed.
        
        Parameters:
            waypoint: this is the new waypoint to be updated         
        """
        self.current_waypoint[id_] = np.array(waypoint) * 100.0


    def show_frame(self, frame, rgb_frame, window_name="Frame"):
        """
        Displays frame for debugging purpose.
        
        Parameters:
            frame: edited RGB Frame with texts
            rgb_frame: original RGB Frame 
            window_name: name of the window where the frame is displayed
        
        Returns:
            None
        """

        for i,(key,pos) in enumerate(self.current_waypoint.items()):
            if pos is not None:
                cv2.putText(frame, f"Goal {i}(m): [{round(pos[0]/100.0, 2)}, {round(pos[1]/100.0, 2)}, {round(pos[2]/100.0, 2)}]", (50 ,50*i+ 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
        
        for i,(key,pos) in enumerate(self.estimated_pose_all.items()):
            if pos is not None:
                cv2.putText(frame, f"Current Estimate {i}(m): [{round(pos[0],2)}, {round(pos[1],2)}, {round(pos[2],2)}]", (50, 150 + 50*i), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
        
        yaw = [round(Rotation.from_rotvec(self.cam_rvecs[self.required_marker_id[0]]).as_euler('xyz', degrees=True)[2], 2), round(Rotation.from_rotvec(self.cam_rvecs[self.required_marker_id[1]]).as_euler('xyz', degrees=True)[2], 2)]
        cv2.putText(frame, f"Yaw0, Yaw1 (deg): [{yaw[0],yaw[1]}]", (50, 250), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
        
        camera_yaw = round(Rotation.from_rotvec(self.cam_rvec).as_euler('xyz', degrees=True)[2], 2)
        camera_roll = round(self.imu_calib_data[0] * 180.0 / np.pi, 2)
        camera_pitch = round(self.imu_calib_data[1] * 180.0 / np.pi, 2)
        cv2.putText(frame, f"Cam RPY (deg): [{camera_roll, camera_pitch, camera_yaw}]", (50, 300), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
        
        for i, (key,pos) in enumerate(self.current_midpoint.items()):
            if pos is not None:
                m = math.tan(yaw[i]*np.pi/180 )
                c = pos[1] - m * (pos[0])
                if abs(m) > 1000:
                    cv2.line(frame, (int(0), int(pos[1])), (int(self.rgb_res[1]), int(pos[1])), (255, 0, 0), 3)    
                else:
                    cv2.line(frame, (int(0), int(c)), (int(self.rgb_res[1]), int(m * self.rgb_res[1] + c)), (255, 0, 0), 3)

        if not(self.avg_fps is None):
            cv2.putText(frame, f"Average FPS: {round(self.avg_fps,2)}", (50, 350), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)

        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.imshow(window_name, frame)

        cv2.namedWindow("RGB Image", cv2.WINDOW_NORMAL)
        cv2.imshow("RGB Image", rgb_frame)
        key = cv2.waitKey(1)


    

    def estimate_uncalib_pose(self, marker_corners):
        """
        Returns attitude of Aruco to estimate initial yaw (used by calibrate_yaw() method).
        
        Parameters:
            marker_corners: corners of the detected aruco tag            
        
        Returns:
            uncalib_rot: returns attitude of Aruco
        """
        rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
                [marker_corners.astype(np.float32)], self.marker_size, self.cam_matrix, self.dist_coef
            )

        aruco_rot = Rotation.from_rotvec(rVec[0, 0, :]).as_matrix()
        aruco_rot = self.rp_correction @ aruco_rot
        aruco_rot = np.linalg.pinv(aruco_rot)

        uncalib_rot = Rotation.from_matrix(aruco_rot).as_euler('xyz')
        return uncalib_rot[2]



    def estimate_pose(self, marker_corners, frame_of_ref="camera"):  
        """
        Estimates the pose of the transform matrix using large ArUco tag.
        
        Parameters:
            marker_corners: list of marker corners
            frame_of_ref: frame of reference. Default value is "camera"
        
        Returns:
            6-DOF pose estimate of aruco marker in world frame    
        """  
        rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
                [marker_corners.astype(np.float32)], self.marker_size, self.cam_matrix, self.dist_coef
            )

        if self.DEBUG:
            print("[RAW ARUCO] (meters)--> X:", tVec[0, 0, 0] / 100.0, ", Y:", tVec[0, 0, 1] / 100.0, "Z:", tVec[0, 0, 2] / 100.0)
            
        if self.camera_config['wandb']['use_wandb']:
            wandb.log({
                'raw_aruco_x': tVec[0, 0, 0] / 100.0,
                'raw_aruco_y': tVec[0, 0, 1] / 100.0,
                'raw_aruco_z': tVec[0, 0, 2] / 100.0
            })

        output = self.rpy_correction @ np.array([tVec[0, 0, 0], tVec[0, 0, 1], tVec[0, 0, 2]])
        output = output / 100.0
        output = output - self.cam_tvec

        return output


    def make_tf_matrix(self, rvec, tvec):
        """
        Creates the transormation matrix using the rotation and translational vectors.
        
        Parameters:
            rvec: contains rotational vectors
            tvec: contains trsnslational vectors
        
        Returns:
            tf: transformation matrix for the camera
        """
        rot_mat = Rotation.from_rotvec(rvec).as_matrix()
        tf = np.eye(4)
        tf[:3, :3] = rot_mat
        tf[:3, 3] = tvec
        return tf

    
    def tf_to_outformat(self, tf):
        """
        Converts the transformation matrix to a list.
        
        Parameters:
            tf: transformation matrix
        
        Returns:
            out_vec: conversion of the transformation matrix into list
        """
        out_vec = np.zeros((6, 1))
        out_vec[3:, 0] = Rotation.from_matrix(tf[:3, :3]).as_euler('xyz')
        out_vec[:3, 0] = tf[:3, 3]
        return out_vec


    def outformat_to_tf(self, input):
        """
        Converts the list to the transformation matrix. 
        
        Parameters:
            out_vec: conversion of the transformation matrix into list            
        
        Returns:            
            tf: transformation matrix
        """
        tf = np.eye(4)
        tf[:3, :3] = Rotation.from_euler('xyz', input[3:, 0]).as_matrix()
        tf[:3, 3] = input[:3, 0]
        return tf


    def depth_from_marker(self, depth_frame, marker_corners, kernel_size=1):
        """
        Finds depth from the estimated pose obtained from the ArUco tag.
        Parameters:
            depth_frame:
            marker_corners: contains the list of aruco marker corners
            kernel_size: contains the size of the kernel
        Returns:
            depths: depth of the point filtered using median filter
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
        
        depths = list(set(depths))
        depths.sort()
        num_values = len(depths)
        if num_values % 2 == 0:
            return (depths[int(num_values / 2.0)] + depths[int(num_values / 2.0) - 1]) / 2.0
        else:
            return depths[int(num_values / 2.0) - 1]

    def cam_init(self):
        """
        Initializes camera and starts ArUco detection.
        Parameters:
            None
        Returns:
            None
        """
        self.init_realsense()
        self.init_aruco_detector()
        
    # def calibrate_yaw(self):
    #         """
    #         Updates the estimated initial yaw in the rotation vector, by taking median of multiple yaw readings.
            
    #         Parameters:
    #             None            
            
    #         Returns:
    #             None
    #         """
    #         max_iters = 100
    #         num_calib_frames = 0
    #         rvec_uncalib = []
    #         while True:
    #             aligned_frames = self.get_frames()    
    #             color_frame = self.extract_rgb(aligned_frames)
    #             depth_frame = self.extract_depth(aligned_frames)
    #             if not depth_frame or not color_frame:
    #                 continue
                
    #             color_img = self.to_image(color_frame)
    #             marker_corners = self.detect_marker(color_img)
                
    #             if marker_corners is None:
    #                 pass
    #             elif type(marker_corners) is str:
    #                 pass
    #             else:
    #                 num_calib_frames += 1
    #                 rvec_uncalib.append(self.estimate_uncalib_pose(marker_corners))
    #                 if(num_calib_frames >= max_iters):
    #                     print("Yaw Calibrated ")
    #                     break   

    #         rvec_uncalib.sort()
    #         self.raw_calib_yaw = rvec_uncalib[int(len(rvec_uncalib) / 2.0)]
    #         temp_ = Rotation.from_euler('xyz', np.array([0.0, 0.0, self.raw_calib_yaw])).as_rotvec()[2]
    #         yawTemp_ = (np.pi+temp_+np.pi/2)%(2*np.pi)-np.pi
    #         self.cam_rvec = np.array([0.0, 0.0, yawTemp_])
    #         print(f"YAW Value from Calibration: {self.cam_rvec}")
    
    def calibrate_yaw(self):
        """
        Updates the estimated initial yaw in the rotation vector, by taking median of multiple yaw readings.
        
        Parameters:
            None            
        
        Returns:
            None
        """

        max_iters = self.camera_config["camera"][self.camera_id]["extrinsics"]["max_iters"]
        yaw_error_threshold = self.camera_config["yaw_error_threshold"]
        
        num_calib_frames = 0
        
        # For marker 0
        rvec_uncalib = []
        while True:
            aligned_frames = self.get_frames()    
            color_frame = self.extract_rgb(aligned_frames)
            depth_frame = self.extract_depth(aligned_frames)
            if not depth_frame or not color_frame:
                continue
            
            color_img = self.to_image(color_frame)
            marker_corners_all = self.detect_marker(color_img)
            marker_corners = None
            for (key, mc) in marker_corners_all.items():
                if key == self.required_marker_id[0]:
                    marker_corners = mc
                
            if marker_corners is None:
                pass
            elif type(marker_corners) is str:
                pass
            else:
                mid_point = np.sum(marker_corners[0], 0) / 4.0
                mid_point = (mid_point + 0.5).astype(np.int32)

                self.current_midpoint[self.required_marker_id[0]] = mid_point.copy()
                
                num_calib_frames += 1
                rvec_uncalib.append(self.estimate_uncalib_pose(marker_corners))
                if(num_calib_frames >= max_iters):
                    print(f"Yaw Calibrated for drone 1 with ID: {self.required_marker_id[0]}")
                    break   

        rvec_uncalib.sort()
        self.raw_calib_yaws[self.required_marker_id[0]] = rvec_uncalib[int(len(rvec_uncalib) / 2.0)]
        temp_ = Rotation.from_euler('xyz', np.array([0.0, 0.0, self.raw_calib_yaws[self.required_marker_id[0]]])).as_rotvec()[2]
        yawTemp_ = (np.pi+temp_+np.pi/2)%(2*np.pi)-np.pi
        self.cam_rvecs[self.required_marker_id[0]] = np.array([0.0, 0.0, yawTemp_])
        print(f"YAW Value from Calibration: {self.cam_rvecs}")
            
        
        while True:
            num_calib_frames = 0
        
        # For marker 1
            rvec_uncalib = []
            while True:
                aligned_frames = self.get_frames()    
                color_frame = self.extract_rgb(aligned_frames)
                depth_frame = self.extract_depth(aligned_frames)
                if not depth_frame or not color_frame:
                    continue
                
                color_img = self.to_image(color_frame)
                marker_corners_all = self.detect_marker(color_img)
                marker_corners = None
                for (key, mc) in marker_corners_all.items():
                    if key == self.required_marker_id[1]:
                        marker_corners = mc
                    
                if marker_corners is None:
                    pass
                elif type(marker_corners) is str:
                    pass
                else:
                    mid_point = np.sum(marker_corners[0], 0) / 4.0
                    mid_point = (mid_point + 0.5).astype(np.int32)
                    self.current_midpoint[self.required_marker_id[1]] = mid_point.copy()
                    num_calib_frames += 1
                    rvec_uncalib.append(self.estimate_uncalib_pose(marker_corners))
                    if(num_calib_frames >= max_iters):
                        print(f"Yaw Calibrated for drone 1 with ID: {self.required_marker_id[1]}")
                        break   

            rvec_uncalib.sort()
            self.raw_calib_yaws[self.required_marker_id[1]] = rvec_uncalib[int(len(rvec_uncalib) / 2.0)]
            temp_ = Rotation.from_euler('xyz', np.array([0.0, 0.0, self.raw_calib_yaws[self.required_marker_id[1]]])).as_rotvec()[2]
            yawTemp_ = (np.pi+temp_+np.pi/2)%(2*np.pi)-np.pi
            self.cam_rvecs[self.required_marker_id[1]] = np.array([0.0, 0.0, yawTemp_])
            print(f"YAW Value from Calibration: {self.cam_rvecs}")
            
            if np.linalg.norm(self.cam_rvecs[self.required_marker_id[0]] - self.cam_rvecs[self.required_marker_id[1]])<yaw_error_threshold:
                break
        
        self.cam_rvec = self.cam_rvecs[self.required_marker_id[0]]        
            
        
    def cam_process(self):
        """
        Processes video frames to detect and locate Aruco markers, find depth value using the depth frame and then updating the camera data queue with the estimated readings.
        
        Parameters:
            cam_queue: a queue that is storing camera data where the latest reading is accessed by its last element            
        
        Returns:
            None
        """ 
        flag = 0
        aligned_frames = self.get_frames()    
        color_frame = self.extract_rgb(aligned_frames)
        depth_frame = self.extract_depth(aligned_frames)

        if not depth_frame or not color_frame:
            return

        self.current_time = time.time()
        if not(self.last_time is None):
            time_diff = self.current_time - self.last_time
            if(time_diff != 0.0):
                self.fps_moving_window.append(1/time_diff)
                self.fps_moving_window[-self.fps_moving_window_size:]
                self.avg_fps = np.mean(self.fps_moving_window)
            
        self.last_time = self.current_time

        if self.DEBUG:
            if not(self.avg_fps is None):
                print(f"-----------------------------------------------------Average FPS: ", self.avg_fps)
            

        color_img = self.to_image(color_frame)
        marker_corners_all = self.detect_marker(color_img)
        for (key,marker_corners) in marker_corners_all.items():
            if marker_corners is None:
                self.counter += 1
                if self.counter >= 30:
                    flag = 2
                    # cam_queue.append([flag])
                    self.counter = 0
                    self.estimated_pose_all[key] = flag
                    continue
            elif type(marker_corners) == type("None"):
                flag = 1
                # cam_queue.append([flag])
                self.estimated_pose_all[key] = flag
                continue
            else:
                self.counter = 0
                aruco_pose = self.estimate_pose(marker_corners)
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

                self.current_midpoint[key] = mid_point.copy()
                point_from_rs = rs.rs2_deproject_pixel_to_point(_intrisics, [mid_point[0], mid_point[1]], z_from_realsense)
                
                if self.DEBUG:
                    print(f"[RAW REALSENSE] {key} (meters)--> X:", point_from_rs[0], "Y: ", point_from_rs[1], "Z: ", point_from_rs[2])


                if self.camera_config['wandb']['use_wandb']:
                    wandb.log({
                        f'raw_realsense_x_{key}': point_from_rs[0],
                        f'raw_realsense_y_{key}': point_from_rs[1],
                        f'raw_realsense_z_{key}': point_from_rs[2], 
                    })    

                point_from_rs[:3] = self.rpy_correction @ np.array([point_from_rs[0], point_from_rs[1], point_from_rs[2]])
                point_from_rs[:3] = point_from_rs[:3] - np.array(self.camera_extrinsic['realsense_origin'])


                aruco_pose[0] = -aruco_pose[0]
                point_from_rs[0] = -point_from_rs[0]
                z_from_realsense = point_from_rs[2]


                if self.DEBUG:
                    print(f"[ARUCO] {key} (meters)--> X:", aruco_pose[0], ", Y:", aruco_pose[1], ", Z:", aruco_pose[2])
                    print(f"[REALSENSE] {key} (meters)--> X: ", point_from_rs[0], ", Y: ", point_from_rs[1], "Z: ", point_from_rs[2])


                if self.camera_config['wandb']['use_wandb']:
                    wandb.log({
                        f'aruco_x_{key}': aruco_pose[0],
                        f'aruco_y_{key}': aruco_pose[1],
                        f'aruco_z_{key}': aruco_pose[2],

                        f'realsense_x_{key}': point_from_rs[0],
                        f'realsense_y_{key}': point_from_rs[1],
                        f'realsense_z_{key}': point_from_rs[2],
                    })
                
                if self.camera_config['use_aruco_xy']:
                    # cam_queue.append([self.current_time, aruco_pose, z_from_realsense])
                    estimated_pose = [aruco_pose[0], aruco_pose[1], z_from_realsense]                
                else:
                    # cam_queue.append([self.current_time, point_from_rs, z_from_realsense])
                    estimated_pose = [point_from_rs[0], point_from_rs[1], point_from_rs[2]]
                
                self.estimated_pose_all[key] = estimated_pose
                
        return self.estimated_pose_all
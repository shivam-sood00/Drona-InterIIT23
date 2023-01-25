import pyrealsense2 as rs
import numpy as np
import cv2
from cv2 import aruco
import time
import os
from scipy.spatial.transform import Rotation
import csv
import math

# cv2.namedWindow("RGB Image", cv2.WINDOW_NORMAL)

class VisionPipeline():

    def __init__(self,
                 depth_res=(720, 1280),
                 rgb_res=(720, 1280),
                 align_to="rgb",
                 marker_size=3.75,
                 marker_type=aruco.DICT_4X4_50,
                 required_marker_id=1,
                 calib_file_path="../calib_data/MultiMatrix.npz",
                 debug=0,
                 padding=50,
                 do_tracking=True,
                 calib_yaw_at_start=True,
                 imu_calib_data=[0.0, 0.0, 0.0]) -> None:


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

        self.do_tracking = do_tracking
        self.last_detected_marker = None
        self.tracking_area_th = 160
        self.tracking_point_th = 12

        self.estimated_pose = None

        self.current_waypoint = None

        self.current_midpoint = None

        self.calib_yaw_at_start = calib_yaw_at_start
        self.imu_calib_data = imu_calib_data

        self.avg_fps = None
        
        pass


    def init_realsense(self):
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
    
    
    def find_area(self, corners):
        corners = corners[0]
        area = (corners[0][0] * corners[1][1] + corners[1][0] * corners[2][1] + corners[2][0] * corners[3][1] + corners[3][0] * corners[0][1])
        area = area - (corners[1][0] * corners[0][1] + corners[2][0] * corners[1][1] + corners[3][0] * corners[2][1] + corners[0][0] * corners[3][1])
        return area

    
    def detect_marker(self, frame):
        # frame = cv2.medianBlur(frame, 3)
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        marker_corners, marker_IDs, reject = aruco.detectMarkers(
            gray_frame, self.marker_dict, parameters=self.param_markers
        )

        if self.DEBUG:
            frame = self.plot_markers(frame, marker_corners, marker_IDs)
            rgb_frame = self.plot_markers(frame.copy(), marker_corners, marker_IDs)
            # frame = self.plot_rej_markers(frame, reject)
            # cv2.imwrite(f"frames/{time.time()}.jpg", frame)
            # self.show_frame(frame)

        if marker_IDs is None:
            
            if (self.do_tracking == True):
                
                if (reject is None) or (self.last_detected_marker is None):
                    if self.DEBUG:
                        frame = self.plot_rej_markers(frame, reject)
                        rgb_frame = self.plot_rej_markers(rgb_frame, reject)
                        self.show_frame(frame, rgb_frame)
                        print("NO marker detected")
                    return None
                
                else:
                    last_area = self.find_area(self.last_detected_marker)
                    last_center = np.mean(self.last_detected_marker[0], 0) / 4.0
                    
                    for i, reject_corners in enumerate(reject):
                        new_center = np.mean(reject_corners[0], 0) / 4.0
                        new_area = self.find_area(reject_corners)
                        
                        if((abs(new_area - last_area) <= self.tracking_area_th) and (math.sqrt(np.sum((new_center - last_center) ** 2)) <= self.tracking_point_th)):
                            self.last_detected_marker = reject_corners.copy()
                            
                            if self.DEBUG:
                                frame = cv2.polylines(frame, [reject_corners.astype(np.int32)], True, (255, 0, 0), 4, cv2.LINE_AA)
                                rgb_frame = cv2.polylines(rgb_frame, [reject_corners.astype(np.int32)], True, (255, 0, 0), 4, cv2.LINE_AA)
                                self.show_frame(frame, rgb_frame)
                                
                            return reject_corners.astype(np.int32)
                        
                        else:
                            
                            if self.DEBUG:
                                print("NO TRACKING: ", new_area, new_center)
                                print("LAST: ", last_area, last_center)


                    if self.DEBUG:
                        frame = self.plot_rej_markers(frame, reject)
                        rgb_frame = self.plot_rej_markers(rgb_frame, reject)
                        print("NO marker detected")
                        self.show_frame(frame, rgb_frame)
                    
                    self.last_detected_marker = None
                    return None
            
            else:
                if (reject is None):
                    if self.DEBUG:
                        frame = self.plot_rej_markers(frame, reject)
                        rgb_frame = self.plot_rej_markers(rgb_frame, reject)
                        self.show_frame(frame, rgb_frame)
                        print("NO marker detected")
                    return None
                        # print(reject_corners.shape)
                    
        
        if self.DEBUG:
            frame = self.plot_rej_markers(frame, reject)
            rgb_frame = self.plot_rej_markers(rgb_frame, reject)
            self.show_frame(frame, rgb_frame)            
                    

        for i, id_ in enumerate(marker_IDs):
            
            if id_ == self.required_marker_id:
                # print(marker_corners[i])
                mid_point = np.sum(marker_corners[i][0], 0) / 4.0
                # print(mid_point)
                if (mid_point[0] >= self.rgb_res[1] - self.padding) or (mid_point[0] <= self.padding) or (mid_point[1] >= self.rgb_res[0] - self.padding) or (mid_point[1] <= self.padding):
                    return "None"

                self.last_detected_marker = marker_corners[i].copy()
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

    def plot_rej_markers(self, frame, marker_corners):
        frame = frame.copy()
        for i, corners in enumerate(marker_corners):
            frame = cv2.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 0), 4, cv2.LINE_AA)

        return frame


    def update_waypoint(self, waypoint):
        self.current_waypoint = waypoint
        # self.current_waypoint[0] = self.current_waypoint[0]
        self.current_waypoint = np.array(self.current_waypoint) * 100.0


    def show_frame(self, frame, rgb_frame, window_name="Frame"):

        if (self.current_waypoint is None):
            pass
        else:

            # point_ = np.array([self.current_waypoint[0], self.current_waypoint[1], self.current_waypoint[2], 1]).reshape((4, 1))

            # correction_tf = self.make_tf_matrix(rvec=Rotation.from_euler('xyz', np.array(self.imu_calib_data)).as_rotvec(), tvec=np.array([0.0, 0.0, 0.0])) #  -0.00417288, 0.00310284, 0
            # correction_tf = np.linalg.pinv(correction_tf)
            
            # translation_tf = self.make_tf_matrix(rvec=Rotation.from_euler('xyz', np.array([0, 0, 0])).as_rotvec(), tvec=np.array([self.cam_tvec[0], self.cam_tvec[1], 2.858]))
            
            # point_in_cam = correction_tf @ translation_tf @ point_
            # point_in_cam[:3, :] = point_in_cam[:3, :] / point_in_cam[3, 0]

            # temp_point = point_in_cam[:3, :]
            # temp_point = self.cam_matrix @ temp_point
            # temp_point[:2, :] = temp_point[:2, :] / temp_point[2, 0]

            # cv2.circle(frame, (int(temp_point[0, 0] + 0.5), int(temp_point[1, 0] + 0.5)), 7, (227, 3, 252), -1)
            cv2.putText(frame, f"Goal: [{self.current_waypoint[0]/100.0}, {self.current_waypoint[1]/100.0}, {self.current_waypoint[2]/100.0}]", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
        
        if self.estimated_pose is None:
            pass
        else:
            cv2.putText(frame, f"Current Estimate: [{self.estimated_pose[0]}, {self.estimated_pose[1]}, {self.estimated_pose[2]}]", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(frame, f"YAW: [{Rotation.from_rotvec(self.cam_rvec).as_euler('xyz', degrees=True)}]", (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
        m = math.tan(self.cam_rvec[2])

        if self.current_midpoint is None:
            pass
        else:
            # print("PLOTLINE")
            c = self.current_midpoint[1] - m * (self.current_midpoint[0])
            cv2.line(frame, (int(0), int(c)), (int(self.rgb_res[1]), int(m * self.rgb_res[1] + c)), (255, 0, 0), 3, cv2.LINE_8)

        if not(self.avg_fps is None):
            cv2.putText(frame, f"Average FPS: {self.avg_fps}", (50, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)

        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.imshow(window_name, frame)

        cv2.namedWindow("RGB Image", cv2.WINDOW_NORMAL)
        cv2.imshow("RGB Image", rgb_frame)


    

    def estimate_uncalib_pose(self, marker_corners):
        rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
                [marker_corners.astype(np.float32)], self.marker_size, self.cam_matrix, self.dist_coef
            )


        # print(f"ARUCO ANGLE: {Rotation.from_rotvec(rVec[0, 0, :]).as_euler('xyz')}")
        tf = self.make_tf_matrix(rVec[0, 0, :], tVec[0, 0, :])
        
        correction_tf = self.make_tf_matrix(rvec=Rotation.from_euler('xyz', np.array(self.imu_calib_data)).as_rotvec(), tvec=np.array([0.0, 0.0, 0.0]))
        tf = correction_tf @ tf
        
        # translation_tf = self.make_tf_matrix(rvec=Rotation.from_euler('xyz', np.array([0, 0, 0])).as_rotvec(), tvec=self.cam_tvec)
        # translation_tf = np.linalg.pinv(translation_tf)
        # tf = translation_tf @ tf

        tf = np.linalg.pinv(tf)

        uncalib_rot = Rotation.from_matrix(tf[:3, :3]).as_euler('xyz')
        # uncalib_rot[0] = 0.0
        # uncalib_rot[1] = 0.0
        # print("ARUCO ANGLE: ", uncalib_rot)

        return uncalib_rot[2]



    def estimate_pose(self, marker_corners, frame_of_ref="camera"):
        #print(marker_corners)
        #print(self.cam_matrix)
        #print(self.dist_coef)
        
        rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
                [marker_corners.astype(np.float32)], self.marker_size, self.cam_matrix, self.dist_coef
            )
        # print("RVec:",rVec,"Tvec:",tVec)
        tf = self.make_tf_matrix(np.array([0, 0, 0]), tVec[0, 0, :])
        if frame_of_ref == "camera":
            
            correction_tf = self.make_tf_matrix(rvec=Rotation.from_euler('xyz', np.array(self.imu_calib_data)).as_rotvec(), tvec=np.array([0.0, 0.0, 0.0]))
            tf = correction_tf @ tf
            
            # self.cam_rvec = Rotation.from_euler('xyz', np.array([0.0, 0.0, 0.14])).as_rotvec()
            translation_tf = self.make_tf_matrix(rvec=self.cam_rvec, tvec=self.cam_tvec)
            # translation_tf = self.make_tf_matrix(rvec=np.array([0.0, 0.0, 0.0]), tvec=self.cam_tvec)
            translation_tf = np.linalg.pinv(translation_tf)
            tf = translation_tf @ tf
            pass
        else:        
            tf = self.cam_tf @ tf
        return self.tf_to_outformat(tf) / 100.0


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

    def cam_init(self):
        self.init_realsense()
        self.init_aruco_detector()
        
    def cam_process(self, cam_queue):
        print(self.calib_yaw_at_start)
        # exit()
        if self.calib_yaw_at_start:
        #### Find drone yaw
            max_iters = 100
            num_calib_frames = 0
            rvec_uncalib = []
            while True:
                aligned_frames = self.get_frames()    
                color_frame = self.extract_rgb(aligned_frames)
                depth_frame = self.extract_depth(aligned_frames)

                if not depth_frame or not color_frame:
                    continue
                
                color_img = self.to_image(color_frame)
                marker_corners = self.detect_marker(color_img)
                
                if marker_corners is None:
                    pass
                elif type(marker_corners) is str:
                    pass
                else:
                    num_calib_frames += 1
                    rvec_uncalib.append(self.estimate_uncalib_pose(marker_corners))
                    # if rvec_uncalib is None:
                    #     rvec_uncalib = pipeline.estimate_uncalib_pose(marker_corners)
                    # else:
                    #     rvec_uncalib += pipeline.estimate_uncalib_pose(marker_corners)
                    
                    if(num_calib_frames >= max_iters):
                        print("yaw Caliberated ")
                        break  

            rvec_uncalib.sort()
            temp_ = Rotation.from_euler('xyz', np.array([0.0, 0.0, rvec_uncalib[int(len(rvec_uncalib) / 2.0)]])).as_rotvec()[2]
            self.cam_rvec = np.array([0.0, 0.0, temp_+np.pi/2])
            print(f"YAW Value from Calibration: {self.cam_rvec}")
        
        else:
            self.cam_rvec = np.array([0.0, 0.0, 0.0])
            

        last_time = None
        current_time = None
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
            if not(last_time is None):
                time_diff = current_time - last_time
                if(time_diff != 0.0):
                    if self.avg_fps is None:
                        self.avg_fps = 1/(time_diff)
                    else:
                        self.avg_fps = (self.avg_fps + (1 / time_diff)) / 2.0
            
            last_time = current_time

            if self.DEBUG:
                if not(self.avg_fps is None):
                    print(f"Average FPS: ", self.avg_fps)
            

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

                self.current_midpoint = mid_point.copy()
                
                #print(f"[{current_time}]: Z from REALSENSE: ", z_from_realsense)
                # if aruco_pose[0] >= x_threshold or aruco_pose[1] >= y_threshold:
                #     flag = 1
                #     cam_queue.append([flag])
                # else:
                #     flag = 0
                    # print("appending")
                new_tf = self.make_tf_matrix(rvec=Rotation.from_euler('xyz', np.array(self.imu_calib_data)).as_rotvec(), tvec=np.array([0.0, 0.0, 0.0]))
                
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
                self.estimated_pose = [aruco_pose[0][0], aruco_pose[1][0], z_from_realsense]

                # data  = [dt,current_time,z_from_realsense]
                # data.extend(aruco_pose.T[0].tolist())
            # cv2.namedWindow("RGB Image", cv2.WINDOW_NORMAL)
            # cv2.imshow("RGB Image",color_img)
                

            key = cv2.waitKey(1)
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                
                break
    
    

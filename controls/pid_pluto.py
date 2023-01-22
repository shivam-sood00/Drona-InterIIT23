import numpy as np
class PID():
    """
    Inputs -> current state (x, y, z)
    Outputs -> Thrust, Roll, Pitch (in 900-2100 range)
    [Kp, KD, KI] are the PID gains 

    """
    def __init__(self,config,droneNo):
        self.K_thrust_hover = np.array(config.get(droneNo,"K_thrust_hover").split(','),dtype=np.float64).reshape(3,1)
        self.K_roll_hover = np.array(config.get(droneNo,"K_roll_hover").split(','),dtype=np.float64).reshape(3,1)
        self.K_pitch_hover = np.array(config.get(droneNo,"K_pitch_hover").split(','),dtype=np.float64).reshape(3,1)
        self.K_yaw_hover = np.array(config.get(droneNo,"K_yaw_hover").split(','),dtype=np.float64).reshape(3,1)
        
        self.K_thrust_way = np.array(config.get(droneNo,"K_thrust_way").split(','),dtype=np.float64).reshape(3,1)
        self.K_roll_way = np.array(config.get(droneNo,"K_roll_way").split(','),dtype=np.float64).reshape(3,1)
        self.K_pitch_way = np.array(config.get(droneNo,"K_pitch_way").split(','),dtype=np.float64).reshape(3,1)
        self.K_yaw_way = np.array(config.get(droneNo,"K_yaw_way").split(','),dtype=np.float64).reshape(3,1)
        
        self.steady_state_err_hover = np.array(config.get(droneNo,"steady_state_err_hover").split(','),dtype=np.float64).reshape(2,1)
        self.steady_state_err_way = np.array(config.get(droneNo,"steady_state_err_way").split(','),dtype=np.float64).reshape(2,1)
        # print(self.K_pitch,self.K_roll,self.K_pitch,self.K_yaw)
        self.cur_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape(6,1) # x,y,z,yaw
        self.prev_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape(6,1)
        self.target_pose = np.array([0.0, 0.0, 0.0]).reshape(3,1)
        self.waypoint = np.array([0.0, 0.0, 0.0]).reshape(3,1)
        self.backward_pitch_scale = 1.0                                    #Unsymmetric dynamics due to arUco
        self.zero_yaw = None
        self.useWay = False
        self.int_moving_win_len = int(config.get(droneNo,"int_moving_win_len"))
        self.diff_moving_win_len = int(config.get(droneNo,"diff_moving_win_len"))
        self.total_window = max(self.int_moving_win_len,self.diff_moving_win_len)
        
        self.xy_thresh = float(config.get("Thresholds","xy"))
        self.vel_thresh = float(config.get("Thresholds","vel"))
        self.z_thresh = float(config.get("Thresholds","z"))
        self.reset()
        
    """
    e, e_dot, e_integral
    """
    def reset(self):
        self.err_thrust = np.array([0.0, 0.0, 0.0]).reshape(3,1)
        self.err_roll = np.array([0.0, 0.0, 0.0]).reshape(3,1)
        self.err_pitch = np.array([0.0, 0.0, 0.0]).reshape(3,1)
        self.err_roll_with_sse = 0.0
        self.err_pitch_with_sse = 0.0
        self.err_yaw = np.array([0.0, 0.0, 0.0]).reshape(3,1)
        self.prev_err = np.array([0.0, 0.0, 0.0, 0.0]).reshape(4,1)      #Thrust, Roll, Pitch, Yaw for Derivative term
        self.prev_err_list = [[],[],[],[]] #thrust, roll, pitch, yaw

    def calc_err(self):
        # self.update_target_waypoint()
        # print(self.waypoint)
        
        self.err_thrust[0] = self.target_pose[2] - self.cur_pose[2]
        self.err_roll[0] = self.target_pose[1] - self.cur_pose[1]
        self.err_pitch[0] = self.target_pose[0] - self.cur_pose[0]
        self.err_yaw[0] = self.zero_yaw - self.cur_pose[3]
        
        self.prev_err_list[0].append(self.err_thrust[0])
        self.prev_err_list[1].append(self.err_roll[0])
        self.prev_err_list[2].append(self.err_pitch[0])
        self.prev_err_list[3].append(self.err_yaw[0])

        for i in range(len(self.prev_err_list)):
            self.prev_err_list[i] = self.prev_err_list[i][-self.total_window:]

        diff_frame = min (self.diff_moving_win_len,len(self.prev_err_list[0]))
        self.err_thrust[1] = self.err_thrust[0] - self.prev_err_list[0][-diff_frame]
        self.err_roll[1] = self.err_roll[0] - self.prev_err_list[1][-diff_frame]
        self.err_pitch[1] = self.err_pitch[0] - self.prev_err_list[2][-diff_frame]
        self.err_yaw[1] = self.err_yaw[0] - self.prev_err_list[3][-diff_frame]


        self.err_thrust[2] = np.clip(sum(self.prev_err_list[0][-self.int_moving_win_len:]), -100, 100)        
        self.err_roll[2] = np.clip(sum(self.prev_err_list[1][-self.int_moving_win_len:]), -30, 30)
        self.err_pitch[2] = np.clip(sum(self.prev_err_list[2][-self.int_moving_win_len:]), -30, 30)
        self.err_yaw[2] = np.clip(sum(self.prev_err_list[3][-self.int_moving_win_len:]), -30, 30)
        
        if self.useWay:
            self.err_pitch_with_sse = self.err_pitch[0] - self.steady_state_err_way[0]
            self.err_roll_with_sse = self.err_roll[0] - self.steady_state_err_way[1]
        else:
            self.err_pitch_with_sse = self.err_pitch[0] - self.steady_state_err_hover[0]
            self.err_roll_with_sse = self.err_roll[0] - self.steady_state_err_hover[1]
            

    def update_pos(self,curPose):
        self.prev_pose = self.cur_pose
        self.cur_pose = np.array(curPose).reshape(6,1)

    def set_target_pose(self,point):
        self.target_pose = np.array(point).reshape(3,1)                                           #TODO implement Carrot
        if not self.useWay:
            self.target_pose[:2] += self.steady_state_err_hover[:]
        else:
            self.target_pose[:2] += self.steady_state_err_way[:]

    # def update_target_waypoint(self):
    #     dif = self.target_pose - self.cur_pose[:3]
    #     mag = np.linalg.norm(dif)**0.5
    #     distanceForWayPoint = 0.2
    #     self.useWay = distanceForWayPoint>mag
    #     if self.useWay:
    #         self.waypoint = self.target_pose                                                #TODO Import wp from config
    #     else:
    #         self.waypoint = self.cur_pose[:3] + distanceForWayPoint*(dif/mag)
    #     self.waypoint[2] = self.target_pose[2]
    
    def set_thrust(self):
        self.thrust = np.sum(self.K_thrust_hover * self.err_thrust)      #Elementwise multiplication
        if self.useWay:
            self.thrust = np.sum(self.K_thrust_way * self.err_thrust)
        
        scale = np.clip(1/(np.cos(np.radians(self.cur_pose[-1]))*np.cos(np.radians(self.cur_pose[-2]))), 1, 1.1)
        self.thrust = scale*self.thrust
        self.thrust = 1550 + np.clip(self.thrust, -250, 300)       #TODO tune (Import from config)
        return self.thrust

    def set_thrust_using_dynamics(self):
        """
        Use roll, pitch angles to keep z stable while waypoint navigation
        """
        pass
    
    def set_pitch_and_roll(self):
        # print(type(self.K_roll),type(self.err_roll))
        # print(self.K_roll,self.err_roll)
        roll = np.sum(self.K_roll_hover * self.err_roll)
        pitch = np.sum(self.K_pitch_hover * self.err_pitch)
        if self.useWay:
            roll = np.sum(self.K_roll_way * self.err_roll)
            pitch = np.sum(self.K_pitch_way * self.err_pitch)

        # self.pitch = pitch
        # self.roll= roll
        yaw_ref = np.radians(self.cur_pose[3] - self.zero_yaw)
        # print("YAW", self.cur_pose[3])
        # print("CUR", self.cur_pose[3] , self.zero_yaw, "sin", np.sin(yaw_ref), "cos", np.cos(yaw_ref))
        self.roll = roll*np.cos(yaw_ref) - pitch*np.sin(yaw_ref)
        self.pitch = pitch*np.cos(yaw_ref) + roll*np.sin(yaw_ref)  #Coupled dynamics if yaw_ref changes

        # self.pitch = 1500 + np.clip(self.pitch, -150, 150) 
        # self.roll = 1500 - np.clip(self.roll, -150, 150)           #TODO tuned 
        self.pitch = 1500 + np.clip(self.pitch, -250, 250) 
        self.roll = 1500 - np.clip(self.roll, -250, 250)           #TODO tuned 
        return self.pitch, self.roll  
    
    def set_yaw(self):
        self.yaw = np.sum(self.K_yaw_hover * self.err_yaw)
        if self.useWay:
            self.yaw = np.sum(self.K_yaw_way * self.err_yaw)
        self.yaw = 1500 + np.clip(self.yaw, -150, 150)           #TODO tuned 
        return self.yaw


    def failsafe_out_of_camera(self):
        pass

    def aruco_not_detected(self):
        pass
    
    def isReached(self):
        if self.useWay:
            err = (self.target_pose[:2]-self.steady_state_err_way[:2]) - self.cur_pose[:2]
        else:
            err = (self.target_pose[:2]-self.steady_state_err_hover[:2]) - self.cur_pose[:2]
        # err = abs(self.cur_pose[:2] - (self.target_pose[:2]+self.))
        err = abs(err)
        distCond = np.linalg.norm(err)
        velCond = np.linalg.norm(self.cur_pose[:3] - self.prev_pose[:3])/0.04
        print("distCond, velCond, z_err",np.all(err< self.xy_thresh),velCond,self.target_pose[2]-self.cur_pose[2])
        if np.all(err< self.xy_thresh) and velCond < self.vel_thresh and abs(self.target_pose[2]-self.cur_pose[2])<self.z_thresh:
            return True
        return False
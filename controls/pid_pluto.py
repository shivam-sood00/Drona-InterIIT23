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
        # print(self.K_pitch,self.K_roll,self.K_pitch,self.K_yaw)
        self.cur_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape(6,1) # x,y,z,yaw
        self.prev_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape(6,1)
        self.target_pose = np.array([0.0, 0.0, 0.0]).reshape(3,1)
        self.horizon  = 100
        self.data_fr_ma = np.zeros((1,self.horizon))
        self.counter = 0
        self.waypoint = np.array([0.0, 0.0, 0.0]).reshape(3,1)
        self.backward_pitch_scale = 1.0                                    #Unsymmetric dynamics due to arUco
        self.zero_yaw = None
        self.useWay = False
        self.reset()
        
    """
    e, e_dot, e_integral
    """
    def reset(self):
        self.err_thrust = np.array([0.0, 0.0, 0.0]).reshape(3,1)
        self.err_roll = np.array([0.0, 0.0, 0.0]).reshape(3,1)
        self.err_pitch = np.array([0.0, 0.0, 0.0]).reshape(3,1)
        self.err_yaw = np.array([0.0, 0.0, 0.0]).reshape(3,1)
        self.prev_err = np.array([0.0, 0.0, 0.0, 0.0]).reshape(4,1)      #Thrust, Roll, Pitch, Yaw for Derivative term

    def calc_err(self):
        # self.update_target_waypoint()
        # print(self.waypoint)
        
        self.err_thrust[0] = self.target_pose[2] - self.cur_pose[2]
        self.err_thrust[1] = self.err_thrust[0] - self.prev_err[0]
        self.prev_err[0] = self.err_thrust[0]
        self.err_thrust[2] = np.clip(self.err_thrust[2] + self.err_thrust[0], -100, 100)
        
        self.err_roll[0] = self.target_pose[1] - self.cur_pose[1]
        self.err_roll[1] = self.err_roll[0] - self.prev_err[1]
        self.prev_err[1] = self.err_roll[0]
        
        self.data_fr_ma[:,0:self.horizon-1] = self.data_fr_ma[:,1:self.horizon]
        self.data_fr_ma[0,self.horizon-1] = self.err_roll[0]
        estimated = np.sum(self.data_fr_ma, axis=1)     
        
        if self.counter < self.horizon:
            self.counter += 1
        else:
            self.err_roll[2] = estimated[0]

        self.err_pitch[0] = self.target_pose[0] - self.cur_pose[0]
        self.err_pitch[1] = self.err_pitch[0] - self.prev_err[2]
        self.prev_err[2] = self.err_pitch[0]
        self.err_pitch[2] = np.clip(self.err_pitch[2] + self.err_pitch[0], -30, 30)
        
        self.err_yaw[0] = self.zero_yaw - self.cur_pose[3]
        self.err_yaw[1] = self.err_yaw[0] - self.prev_err[3]
        self.prev_err[3] = self.err_yaw[0]
        self.err_yaw[2] = np.clip(self.err_yaw[2] + self.err_yaw[0], -30, 30)

    def update_pos(self,curPose):
        self.prev_pose = self.cur_pose
        self.cur_pose = np.array(curPose).reshape(6,1)

    def set_target_pose(self,point):
        self.target_pose = np.array(point).reshape(3,1)                                           #TODO implement Carrot

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
        distCond = (np.linalg.norm(self.cur_pose[:2] - self.target_pose[:2]))**0.5 
        velCond = (np.linalg.norm(self.cur_pose[:3] - self.prev_pose[:3]))**0.5 
        # print("distCond, velCond, z_err",distCond,velCond,self.cur_pose[2]-self.target_pose[2])
        if distCond < 0.2 and velCond < 0.1 and abs(self.cur_pose[2]-self.target_pose[2])<0.1:
            return True
        return False
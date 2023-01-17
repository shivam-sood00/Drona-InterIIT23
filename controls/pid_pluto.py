import numpy as np
class PID():
    """
    Inputs -> current state (x, y, z)
    Outputs -> Thrust, Roll, Pitch (in 900-2100 range)
    [Kp, KD, KI] are the PID gains 

    """
    def __init__(self,config,droneNo):
        self.K_thrust = np.array(config.get(droneNo,"K_thrust").split(','),dtype=np.float64).reshape(3,1)
        self.K_roll = np.array(config.get(droneNo,"K_roll").split(','),dtype=np.float64).reshape(3,1)
        self.K_pitch = np.array(config.get(droneNo,"K_pitch").split(','),dtype=np.float64).reshape(3,1)
        self.K_yaw = np.array(config.get(droneNo,"K_yaw").split(','),dtype=np.float64).reshape(3,1)
        # print(self.K_pitch,self.K_roll,self.K_pitch,self.K_yaw)
        self.cur_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape(6,1) # x,y,z,yaw
        self.prev_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape(6,1)
        self.target_pose = np.array([0.0, 0.0, 0.0]).reshape(3,1)
        self.waypoint = np.array([0.0, 0.0, 0.0]).reshape(3,1)
        self.backward_pitch_scale = 1.0                                    #Unsymmetric dynamics due to arUco
        self.zero_yaw = None
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
        self.update_target_waypoint()
        
        self.err_thrust[0] = self.waypoint[2] - self.cur_pose[2]
        self.err_thrust[1] = self.err_thrust[0] - self.prev_err[0]
        self.prev_err[0] = self.err_thrust[0]
        self.err_thrust[2] = np.clip(self.err_thrust[2] + self.err_thrust[0], -100, 100)
        
        self.err_roll[0] = self.waypoint[1] - self.cur_pose[1]
        self.err_roll[1] = self.err_roll[0] - self.prev_err[1]
        self.prev_err[1] = self.err_roll[0]
        self.err_roll[2] = np.clip(self.err_roll[2] + self.err_roll[0], -30, 30)

        self.err_pitch[0] = self.waypoint[0] - self.cur_pose[0]
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

    def update_target_waypoint(self):
        dif = self.target_pose - self.cur_pose[:3]
        mag = np.linalg.norm(dif)**0.5
        distanceForWayPoint = 0.2
        if mag<distanceForWayPoint:
            self.waypoint = self.target_pose                                                #TODO Import wp from config
        else:
            self.waypoint = self.cur_pose[:3] + distanceForWayPoint*(dif/mag)
    
    def set_thrust(self):
        self.thrust = np.sum(self.K_thrust * self.err_thrust)      #Elementwise multiplication
        scale = np.clip(1/(np.cos(np.radians(self.cur_pose[-1]))*np.cos(np.radians(self.cur_pose[-2]))), 1, 1.2)
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
        roll = np.sum(self.K_roll * self.err_roll)
        pitch = np.sum(self.K_pitch * self.err_pitch)

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
        self.yaw = np.sum(self.K_yaw * self.err_yaw)
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
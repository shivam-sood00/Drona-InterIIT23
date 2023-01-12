import numpy as np
import yaml
class PID():
    """
    Inputs -> current state (x, y, z)
    Outputs -> Thrust, Roll, Pitch (in 900-2100 range)
    [Kp, KD, KI] are the PID gains 

    """
    def __init__(self):
        self.config = yaml.load(open("controls/config.yaml",'r'),Loader=yaml.FullLoader)
        self.K_thrust = np.array(self.config['K_thrust']).reshape(3,1)
        self.K_roll = np.array(self.config['K_roll']).reshape(3,1)
        self.K_pitch = np.array(self.config['K_pitch']).reshape(3,1)
        self.K_yaw = np.array(self.config['K_yaw']).reshape(3,1)
        self.cur_pose = np.array([0.0, 0.0, 0.0, 0.0]).reshape(4,1) # x,y,z,yaw
        self.target_pose = np.array([0.0, 0.0, 0.0]).reshape(3,1)
        self.backward_pitch_scale = 1.0                                    #Unsymmetric dynamics due to arUco
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
        self.err_thrust[0] = self.target_pose[2] - self.cur_pose[2]
        self.err_thrust[1] = self.err_thrust[0] - self.prev_err[0]
        self.prev_err[0] = self.err_thrust[0]
        self.err_thrust[2] = np.clip(self.err_thrust[2] + self.err_thrust[0], -100, 100)
        

        self.err_roll[0] = self.target_pose[1] - self.cur_pose[1]
        self.err_roll[1] = self.err_roll[0] - self.prev_err[1]
        self.prev_err[1] = self.err_roll[0]
        self.err_roll[2] = np.clip(self.err_roll[2] + self.err_roll[0], -30, 30)

        self.err_pitch[0] = self.target_pose[0] - self.cur_pose[0]
        self.err_pitch[1] = self.err_pitch[0] - self.prev_err[2]
        self.prev_err[2] = self.err_pitch[0]
        self.err_pitch[2] = np.clip(self.err_pitch[2] + self.err_pitch[0], -30, 30)
        
        self.err_yaw[0] = self.config['yaw_setpoint'] - self.cur_pose[3]
        self.err_yaw[1] = self.err_yaw[0] - self.prev_err[3]
        self.prev_err[3] = self.err_yaw[0]
        self.err_yaw[2] = np.clip(self.err_yaw[2] + self.err_yaw[0], -30, 30)

    def update_pos(self,curPose):
        self.cur_pose = np.array(curPose).reshape(4,1)

    def set_target_pose(self,point):
        self.target_pose = np.array(point).reshape(3,1)                                           #TODO implement Carrot

    def set_target_waypoint(self):
        pass                                                #TODO Import wp from config

    def set_thrust(self):
        self.thrust = np.sum(self.K_thrust * self.err_thrust)      #Elementwise multiplication
        self.thrust = 1550 + np.clip(self.thrust, -250, 250)       #TODO tune (Import from config)
        return self.thrust

    def set_roll(self):
        self.roll = np.sum(self.K_roll * self.err_roll)
        self.roll = 1500 + np.clip(self.roll, -150, 150)           #TODO tuned 
        return self.roll
    
    def set_pitch(self):
        self.pitch = np.sum(self.K_pitch * self.err_pitch)
        if self.pitch < 0:
            self.pitch *= self.backward_pitch_scale
        self.pitch = np.clip(self.pitch, -200, 200)         #TODO tuned
        self.pitch = 1500 - self.pitch
        return self.pitch
    
    def set_yaw(self):
        self.yaw = np.sum(self.K_yaw * self.err_yaw)
        self.yaw = 1500 + np.clip(self.yaw, -150, 150)           #TODO tuned 
        return self.yaw


    def failsafe_out_of_camera(self):
        pass

    def aruco_not_detected(self):
        pass
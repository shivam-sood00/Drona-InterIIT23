import numpy as np
from copy import deepcopy
class PID():
	"""
	This is the implementation of the PID controller.
	The current states are the x, y, z coordinates.
    The control variables are thrust, roll, pitch in the range of 900-2100.
    Kp, Kd and Ki are the PID gains required to tune the control variables.
	"""
	def __init__(self,config,droneNo):
		"""
            Initializing the PID gains and the corresponding parameters for error calculation.
            K_thrust_z, K_roll_z, K_pitch_z, K_yaw_z are the PID gains for the motion corresponding to z axis.

            K_thrust_way_x, K_roll_way_x, K_pitch_way_x, K_yaw_way_x are the PID gains 
            for adaptive tuning of the motion corresponding to x axis, based on distance hueristics.

            K_thrust_way_y, K_roll_way_y, K_pitch_way_y, K_yaw_way_y are the PID gains
            for adaptive tuning of the motion corresponding to y axis, based on distance hueristics.

            steady_state_err_way_z, steady_state_err_way_x, steady_state_err_way_y are the steady state errors
            for the motion corresponding to z, x and y axis respectively.

            move : A dictionary which stores the steady state errors for the motion corresponding to x, y and z axis respectively.

            move_K : A dictionary which stores the PID gains for the motion of qaudrotor corresponding to x, y and z axis respectively.

            cur_steady_state : The current steady state error for the motion of quadrotor.
            cur_pos : The current position of the quadrotor.
            cur_vel : The current velocity of the quadrotor.
            target_pos : The target position of the quadrotor.
            waypoint: Waypoint that the quadrotor is currently moving towards.
            zero_yaw: Yaw angle of the quadrotor when it is stationary.
            useway: Boolean variable to check if the quadrotor is moving towards a waypoint or not.
            int_moving_win_len: Length of the moving window for the integral term.
            diff_moving_win_len: Length of the moving window for the derivative term.
            total_win_len: Total length of the moving window.
            xy_thresh: Threshold for the distance between the quadrotor and the waypoint along x and y axis, while stablizing the quadrotor at the waypoint.
            z_thresh: Threshold for the distance between the quadrotor and the waypoint along the z-axis, while stablizing the quadrotor at the waypoint.
            vel_thresh: Threshold for the velocity of the quadrotor along the x y and z axis, while stablizing the quadrotor at the waypoint.
            vel_error: Error in the velocity of the quadrotor along the x y and z axis, while stablizing the quadrotor at the waypoint.
            diff_fn_dict: Dictionary which stores the moving window, low_pass_filter and average functions for the derivative term.
            diff_fn: Selected function from the diff_fn_dict for the derivative term.
            alpha: Parameter for the low_pass_filter function.
        """
		self.K_thrust_z = np.array(config.get(droneNo,"K_thrust_z").split(','),dtype=np.float64).reshape(3,1)
		self.K_roll_z = np.array(config.get(droneNo,"K_roll_z").split(','),dtype=np.float64).reshape(3,1)
		self.K_pitch_z = np.array(config.get(droneNo,"K_pitch_z").split(','),dtype=np.float64).reshape(3,1)
		self.K_yaw_z = np.array(config.get(droneNo,"K_yaw_z").split(','),dtype=np.float64).reshape(3,1)
		
		self.K_thrust_way_x = np.array(config.get(droneNo,"K_thrust_way_x").split(','),dtype=np.float64).reshape(3,1)
		self.K_roll_way_x = np.array(config.get(droneNo,"K_roll_way_x").split(','),dtype=np.float64).reshape(3,1)
		self.K_pitch_way_x = np.array(config.get(droneNo,"K_pitch_way_x").split(','),dtype=np.float64).reshape(3,1)
		self.K_yaw_way_x = np.array(config.get(droneNo,"K_yaw_way_x").split(','),dtype=np.float64).reshape(3,1)
		
		self.K_thrust_way_y = np.array(config.get(droneNo,"K_thrust_way_y").split(','),dtype=np.float64).reshape(3,1)
		self.K_roll_way_y = np.array(config.get(droneNo,"K_roll_way_y").split(','),dtype=np.float64).reshape(3,1)
		self.K_pitch_way_y = np.array(config.get(droneNo,"K_pitch_way_y").split(','),dtype=np.float64).reshape(3,1)
		self.K_yaw_way_y = np.array(config.get(droneNo,"K_yaw_way_y").split(','),dtype=np.float64).reshape(3,1)
		
		self.cur_K_thrust = None
		self.cur_K_roll = None
		self.cur_K_pitch = None
		self.cur_K_yaw = None
		
		self.steady_state_err_way_z = np.array(config.get(droneNo,"steady_state_err_way_z").split(','),dtype=np.float64).reshape(3,1)
		self.steady_state_err_way_x = np.array(config.get(droneNo,"steady_state_err_way_x").split(','),dtype=np.float64).reshape(3,1)
		self.steady_state_err_way_y = np.array(config.get(droneNo,"steady_state_err_way_y").split(','),dtype=np.float64).reshape(3,1)
		self.move = {'x': self.steady_state_err_way_x , 'y': self.steady_state_err_way_y , 'z': self.steady_state_err_way_z }
		self.move_K = {'x': (self.K_thrust_way_x, self.K_roll_way_x, self.K_pitch_way_x, self.K_yaw_way_x) ,
					   'y': (self.K_thrust_way_y, self.K_roll_way_y, self.K_pitch_way_y, self.K_yaw_way_y) ,
					   'z': (self.K_thrust_z, self.K_roll_z, self.K_pitch_z, self.K_yaw_z) }

		self.cur_steady_state = None 
		self.cur_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape(6,1) # x,y,z,yaw
		self.prev_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape(6,1)
		self.target_pose = np.array([0.0, 0.0, 0.0]).reshape(3,1)
		self.waypoint = np.array([0.0, 0.0, 0.0]).reshape(3,1)
		self.backward_pitch_scale = 1.0         #Unsymmetric dynamics due to arUco
		self.zero_yaw = None
		self.int_moving_win_len = int(config.get(droneNo,"int_moving_win_len"))
		self.diff_moving_win_len = int(config.get(droneNo,"diff_moving_win_len"))
		self.total_window = max(self.int_moving_win_len,self.diff_moving_win_len)
		
		self.xy_thresh = float(config.get("Thresholds","xy"))
		self.vel_thresh = float(config.get("Thresholds","vel"))
		self.z_thresh = float(config.get("Thresholds","z"))

		self.vel_error = 0.
		self.final_diff_error = 0.0
		
		self.diff_fn_dict = {'min':np.min, 'avg':np.average, 'lpf':self.low_pass_filter}
		self.diff_fn = self.diff_fn_dict[config.get(droneNo,"diff_fn")]
		self.alpha = float(config.get(droneNo,"alpha_low_pass"))

		self.is_hovering = True
		self.reset()
		
	"""
	e, e_dot, e_integral
	"""
	def reset(self):
		"""
        Reset the error terms.
        err_thrust: Error in thrust.
        err_roll: Error in roll.
        err_pitch: Error in pitch.
        err_roll_with_sse: Bias term corresponding to the steady state error in roll due to external disturbances at the setpoint.
        err_pitch_with_sse: Bias term corresponding to the steady state error in pitch  due to external disturbancesat the setpoint.
        err_thrust_with_sse: Bias term corresponding to the steady state error in thrust due to external disturbances at the setpoint.
        err_yaw: Error in yaw.
        prev_err: Previous error.
        prev_err_list: List of previous errors corresponding to thrust, roll, pitch and yaw respectively.

        """
		self.err_thrust = np.array([0.0, 0.0, 0.0]).reshape(3,1)
		self.err_roll = np.array([0.0, 0.0, 0.0]).reshape(3,1)
		self.err_pitch = np.array([0.0, 0.0, 0.0]).reshape(3,1)
		self.err_roll_with_sse = 0.0
		self.err_pitch_with_sse = 0.0
		self.err_thrust_with_sse = 0.0
		self.err_yaw = np.array([0.0, 0.0, 0.0]).reshape(3,1)
		self.prev_err = np.array([0.0, 0.0, 0.0, 0.0]).reshape(4,1)      #Thrust, Roll, Pitch, Yaw for Derivative term
		self.prev_err_list = [[],[],[],[]] #thrust, roll, pitch, yaw

	def calc_diff_err(self,diff_frame,err_list):
		"""
            Calculating the differential error and storing the differtial error in list of length diff_moving_win_len
            and then storing the corresponding min, average or low pass filter function being called in the diff_fn.
        """
		temp = []
		for i in range(diff_frame):
			temp.append(err_list[-1-i]- err_list[-2-i])

		if len(temp) == 0:
			return 0
		return self.diff_fn(temp)
	
	def calc_err(self):
		"""
        Calculating the error terms for the proportional, integral and derivative gain of the PID controller 
        corresponding to the thrust, roll, pitch and yaw.
        Simultaneously appending the list containing prev_err_list with the current error term for thrust, roll, pitch and yaw.
        Clipping the error term to the range of -100 to 100 for thrust and -30 to 30 for roll, pitch and yaw respectively.
        Calculating the err_pitch_with_sse, err_roll_with_sse and err_thrust_with_sse for the steady state error in pitch, roll and thrust respectively.

        """
		self.err_thrust[0] = self.target_pose[2] - self.cur_pose[2]
		self.err_roll[0] = self.target_pose[1] - self.cur_pose[1]
		self.err_pitch[0] = self.target_pose[0] - self.cur_pose[0]
		self.err_yaw[0] = self.zero_yaw - self.cur_pose[3]
		
		self.prev_err_list[0].append(deepcopy(self.err_thrust[0]))
		self.prev_err_list[1].append(deepcopy(self.err_roll[0]))
		self.prev_err_list[2].append(deepcopy(self.err_pitch[0]))
		self.prev_err_list[3].append(deepcopy(self.err_yaw[0]))

		for i in range(len(self.prev_err_list)):
			self.prev_err_list[i] = self.prev_err_list[i][-self.total_window:]

		diff_frame = min (self.diff_moving_win_len,len(self.prev_err_list[0])-1)	
		self.err_thrust[1] = self.calc_diff_err(diff_frame,self.prev_err_list[0])
		self.err_roll[1] = self.calc_diff_err(diff_frame,self.prev_err_list[1])
		self.err_pitch[1] = self.calc_diff_err(diff_frame,self.prev_err_list[2])
		self.err_yaw[1] = self.calc_diff_err(diff_frame,self.prev_err_list[3])


		self.err_thrust[2] = np.clip(sum(self.prev_err_list[0][-self.int_moving_win_len:]), -100, 100)        
		self.err_roll[2] = np.clip(sum(self.prev_err_list[1][-self.int_moving_win_len:]), -30, 30)
		self.err_pitch[2] = np.clip(sum(self.prev_err_list[2][-self.int_moving_win_len:]), -30, 30)
		self.err_yaw[2] = np.clip(sum(self.prev_err_list[3][-self.int_moving_win_len:]), -30, 30)
		
		self.err_pitch_with_sse = self.err_pitch[0] - self.cur_steady_state[0]
		self.err_roll_with_sse = self.err_roll[0] - self.cur_steady_state[1]
		self.err_thrust_with_sse = self.err_thrust[0] - self.cur_steady_state[2]
			

	def update_pos(self,curPose):
		"""
        Updating the previous position and the current position of the quadrotor respectively.
        """
		self.prev_pose = self.cur_pose
		self.cur_pose = np.array(curPose).reshape(6,1)

	def set_target_pose(self,point,axis):
		"""
        Updating the target position using the carrot approach.
        """
		self.target_pose = np.array(point).reshape(3,1)                                          
		self.cur_steady_state = self.move[axis]
		self.cur_K_thrust , self.cur_K_roll, self.cur_K_pitch, self.cur_K_yaw = self.move_K[axis]
		
		self.target_pose += self.cur_steady_state
	
	def set_thrust(self):
		"""
        Calculating output of the PID controller corresponding to Thrust and clipping it to the range of -250 to 250.
        """
		self.thrust = np.sum(self.cur_K_thrust * self.err_thrust)
		self.thrust = 1525 + np.clip(self.thrust, -250, 300)
		return self.thrust
	
	def set_pitch_and_roll(self):
		"""
        Calculating output of the PID controller corresponding to Pitch and Roll and clipping it to the range of -250 to 250 respectively.
        """	
		roll = np.sum(self.cur_K_roll * self.err_roll)
		pitch = np.sum(self.cur_K_pitch * self.err_pitch)
		

		self.pitch = pitch
		self.roll= roll
		yaw_ref = np.radians(self.cur_pose[3] - self.zero_yaw)
		if self.is_hovering:
			self.pitch = 1500 + np.clip(self.pitch, -25, 25) 
			self.roll = 1500 - np.clip(self.roll, -25, 25)
		else:
			self.pitch = 1500 + np.clip(self.pitch, -25, 25) 
			self.roll = 1500 - np.clip(self.roll, -25, 25)
		return self.pitch, self.roll  
	
	def set_yaw(self):
		"""
        Calculating output of the PID controller corresponding to Yaw and clipping it to the range of -150 to 150 respectively.
        """
		self.yaw = np.sum(self.cur_K_yaw * self.err_yaw)
		self.yaw = 1500 + np.clip(self.yaw, -150, 150)
		return self.yaw
	
	def isReached(self):
		"""
        Boolean function to check if the quadrotor has reached the target pose.
        When the error in velocity and position is less than a threshold value, the quadrotor is considered to have reached the target pose,
        function returns True, else False.
        """
		err = (self.target_pose - self.cur_steady_state ) - self.cur_pose[:3]
		err = abs(err)
		velCond = np.linalg.norm(self.cur_pose[:3] - self.prev_pose[:3])/0.04
		self.vel_error = velCond
		# print("x_err,y_err, velCond, z_err",err[0],err[1],velCond,err[2])
		if self.is_hovering:
			if np.all(err[:2]< (self.xy_thresh+0.1)) and velCond < (self.vel_thresh+0.05) and err[2]<self.z_thresh:
				return True 
		elif np.all(err[:2]< self.xy_thresh) and velCond < self.vel_thresh and err[2]<self.z_thresh:
			return True
		return False

	def low_pass_filter(self, diff_error_list):
		"""
        Filter to smoothen the error signal for PID controller.
        """
		if len(diff_error_list) > 0:
			self.final_diff_error = self.final_diff_error*(1-self.alpha) + self.alpha*diff_error_list[-1]
			return self.final_diff_error
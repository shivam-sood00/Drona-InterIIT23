import numpy as np
from EKF.QuadrotorDynamics import QuadrotorDynamics

class KalmanFilter():
    """
    Class for Extended Kalman Filter
    
    States and Control Inputs are defined as :-
    State X = [x, y, z, x', y', z']
    Control Input U = [Upward Thurst (T - mg), Pitch Torque, Roll Torque, Yaw Torque]
    
    Uses Quadrotor Dynamics to perform Extended Kalman Filtering Estimation using Aruco Position Data and IMU's Acceleration Data.
    """

    def __init__(self,debug = False) -> None:
        """
        self.imu_noise_cov: IMU Noise Covariance Matrix.
        self.aruco_noise_cov: Aruco Noise Covariance Matrix.
        self.imu_noise_bias: Bias for IMU Sensor.
        self.aruco_noise_bias: Bias for the incoming aruco data.
        self.process_noise: Matrix for Modelling Noise in the State Dynamics Model. 
        self.P: Process Noise Covariance.
        self.Q: Model Process Covariance.
        self.X: States of the drone.
        self.drone: An object of QuadrotorDynamics class to get the A, B, C matrix.
        self.imu_H_vel: Sensor Observation Matrix for IMU to update velocity.
        self.imu_H_pos: Sensor Observation Matrix for IMU to update position.
        self.aruco_H_pos:  Sensor Observation Matrix for Aruco to update position.
        """
        self.imu_noise_cov = np.array([[0.29801201,0.0260134,0.03899557],
                              [0.0260134,0.1585327,0.03870672],
                              [0.03899557,0.03870672,0.61070139]])

        self.aruco_noise_cov = np.array([[ 0.5834703,0.00638922,-0.06083379],
                              [ 0.00638922,0.31987258,-0.02961871],
                              [-0.06083379,-0.02961871,0.64220555]]) 


        self.imu_noise_bias = np.zeros(3)
        self.aruco_noise_bias = np.zeros(3)

        self.X = np.array([0.0 , 0, 0, 0, 0, 0],dtype=np.float32)
        self.P = np.zeros((6,6),dtype=np.float32)
        
        self.process_noise = np.zeros(6)
        self.Q = np.eye(6)

        self.drone = QuadrotorDynamics()

        self.imu_H_vel = np.array([
            [ 0, 0, 0, 1.0, 0, 0],
            [ 0, 0, 0, 0, 1, 0 ],
            [ 0, 0, 0, 0, 0, 1 ]],dtype=np.float32)

        self.imu_H_pos = np.array([
            [ 1.0, 0, 0, 0, 0, 0 ],
            [ 0, 1, 0, 0, 0, 0 ],
            [ 0, 0, 1, 0, 0, 0 ]],dtype=np.float32)
        

        self.aruco_H = np.array([
            [ 1.0, 0, 0, 0, 0, 0 ],
            [ 0, 1, 0, 0, 0, 0 ],
            [ 0, 0, 1, 0, 0, 0 ]],dtype=np.float32)
        
        
        

    def apply_system_dynamics(self,imuData, U, dt):
        """
        Part of Prediction Step that predicts states, and is used with IMU Data and Control input to get a better estimate of states.
        
        Parameters:
            imuData: IMU Data
            U: Throttle (As Required by Quadrotor Dynamics)
            dt: Time Elapsed since last update
        """
        
        A = self.drone.getA()        
        B = self.drone.getB(imuData)
        C = self.drone.getC(imuData)

        A_discretized = A * dt + np.eye(self.X.shape[0])
        B_discretized = B * dt
        C_discretized = C * dt

        # Predict State using system dynamics
        self.X = A_discretized @ self.X + B_discretized @ U + C_discretized + self.process_noise 
        
        # Predict Process Noise Covariance
        self.P = A_discretized @ self.P @ A_discretized.T + self.Q



    def update_measurement(self, H, sensor_obs, sensor_noise_bias, sensor_noise_cov):
        """
        Performs the main step of Extended Kalman filtering and updates the state prediction to better estimate the states.
        
        Parameters:
            H: Observation Matrix
            sensor_obs: Sensor Reading
            sensor_noise_bias: Sensor Bias
            sensor_noise_cov: Sensor Noise Covariance
        """
        # Calculate the measurement residual
        measurement_residual = sensor_obs - ((H @ self.X) + (sensor_noise_bias))    

                    
        # Calculate the measurement residual covariance
        S = H @ self.P @ H.T + sensor_noise_cov

        # Calculate the near-optimal Kalman gain
        # We use pseudoinverse since some of the matrices might be non-square or singular.
        K = self.P @ H.T @ np.linalg.pinv(S)
            
        # Calculate an updated state estimate for time k
        self.X = self.X + (K @ measurement_residual)        
        
        # Update the state covariance estimate for time k
        M = K @ H
        self.P = (np.eye(M.shape[0]) - M) @ self.P @ (np.eye(M.shape[0]) - M).T + K @ sensor_noise_cov @ K.T #P_k - (K_k @ H_k @ P_k) 


    def estimate_pose(self, control_inp, sensor_obs, dt):
        """
        Uses Aruco Position and IMU Data, and fuses the data using Extended Kalman Filter to given a better estimate of the states.
        
        Parameters:
            control_inp: Control Input provided to the drone
            sensor_obs: Readings from Aruco and IMU
            dt: Time Elapsed since last update
            
        Returns:
            X: New estimated states after applying EKF
        """
        imuData = sensor_obs["imu"]
        cameraData = sensor_obs["cam"]
        
        # Store Position and Velocity to be updated to current timestep 
        prevPos = self.X[:3]
        prevVel = self.X[3:]

        # Get IMU Calculated Velocity Estimate
        imu_calc_vel = get_velocity(prevVel,imuData,dt)
        
        # Update the Dynamics and Get Expected States, Input as Throttle
        
        # thrust = get_thrust(control_inp[3])  #TODO thrust vs throttle stick mapping
        thrust = control_inp[3] 
        self.apply_system_dynamics(imuData, thrust, dt)

        # Update the current velocity estimate using IMU Calculated Estimate
        self.update_measurement(self.imu_H_vel, np.array(imu_calc_vel), self.imu_noise_bias, self.imu_noise_cov)
        
        # Calculate Position using updated velocity
        imu_calc_pos = get_pos(prevPos,self.X[3:],dt)

        # Then use the calculated velocity to update the position estimate
        self.update_measurement(self.imu_H_pos, np.array(imu_calc_pos), self.imu_noise_bias, self.imu_noise_cov)

        # Then use aruco data to update the position again
        self.update_measurement(self.aruco_H, cameraData, self.aruco_noise_bias, self.aruco_noise_cov)
        

        return self.X  


def get_velocity(prevVelocity,CurrentAcc,phi,theta,psi, dt):
    """
    Returns the current velocity. The previous timestep velocity is taken with respect to world frame and transformed to body frame, where it is updated using acceleration 
    data of the accelerometer and then again transformed to the world frame.
    
    Parameters:
        prevVelocity: Previous timestep velocity in the world frame
        imuData: Current Readings of IMU Data
        
    Returns:
        A list containing
        velocity_x: current x direction velocity in the world frame
        velocity_y: current y direction velocity in the world frame
        velocity_z: current z direction velocity in the world frame 
    """

    a =  np.cos(psi) * np.cos(theta)
    b = -np.cos(phi) * np.sin(psi) + np.cos(psi) * np.sin(phi) * np.sin(theta)
    c =  np.sin(phi) * np.sin(psi) + np.cos(phi) * np.cos(psi) * np.sin(theta)
    d =  np.sin(psi) * np.cos(theta)
    e =  np.cos(phi) * np.cos(psi) + np.sin(phi) * np.sin(psi) * np.sin(theta)
    f = -np.cos(psi) * np.sin(phi) + np.cos(phi) * np.sin(psi) * np.sin(theta)
    g = -np.sin(theta)
    h =  np.cos(theta) * np.sin(phi)
    i = np.cos(phi) * np.cos(theta)

    R = np.array(   [a,  b,  c ],          
                    [d,  e,  f ],          
                    [g,  h,  i ])

    current_body_vel = R.T @ np.array([prevVelocity[3],prevVelocity[4],prevVelocity[5]])
    current_body_vel += np.array([CurrentAcc[0],CurrentAcc[1],CurrentAcc[2]]) * dt
    current_vel = R @ current_body_vel

    velocity_x  = current_vel[0]
    velocity_y  = current_vel[1]
    velocity_z  = current_vel[2]

    return [velocity_x,velocity_y,velocity_z]

def get_angle_rate(imuData):
    """
    The function takes in Gyro Data from IMU Data and gives euler angular rates which is obtained after transformation.
        
    Parameters:
        imuData: Current Readings of IMU Data
        
    Returns:
    	A list containing
        roll_rate: Rate of roll
        pitch_rate: Rate of pitch
        yaw_rate: Rate of yaw
    """
    
    phi = imuData["Roll"]
    theta = imuData["Pitch"]

    a =  np.sin(phi) * np.tan(theta)
    b =  np.cos(phi) * np.tan(theta)
    c =  np.cos(phi)
    d = -np.sin(phi)
    e =  np.sin(phi)/np.cos(theta)
    f =  np.cos(phi)/np.cos(theta)

    R = np.array(   [1,  a,  b ],          
                    [0,  c,  d ],          
                    [0,  e,  f ] )
    
    currentAngleRate =  R @ np.array([imuData["GyroX"],imuData["GyroY"],imuData["GyroZ"]])

    roll_rate = currentAngleRate[0] 
    pitch_rate = currentAngleRate[1]
    yaw_rate = currentAngleRate[2]

    return [roll_rate,pitch_rate,yaw_rate]

def get_pos(prevPos, currentVelocity, dt):
    """   
    The function takes in previous position in world frame and current velocity to return current position, by using Euler Forward Method.
    
    Parameters:
        prevPos: Previous timestep position in the world frame
        currentVelocity: Current velocity in the world frame
        
    Returns:
        Current Position 
    """
    return prevPos + currentVelocity*dt

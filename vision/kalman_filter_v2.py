import numpy as np
from QuadrotorDynamics import QuadrotorDynamics

class KalmanFilter():

    # State X = [x, y, z, x', y', z', roll, pitch, yaw, roll', pitch', yaw']
    # Control Input U = [Upward Thurst (T - mg), Pitch Torque, Roll Torque, Yaw Torque]

    def __init__(self) -> None:

        self.imu_noise_cov = np.array([[0.29801201,0.0260134,0.03899557],
                              [0.0260134,0.1585327,0.03870672],
                              [0.03899557,0.03870672,0.61070139]])

        self.aruco_noise_cov = np.array([[ 0.5834703,0.00638922,-0.06083379],
                              [ 0.00638922,0.31987258,-0.02961871],
                              [-0.06083379,-0.02961871,0.64220555]]) 


        self.imu_noise_bias = np.zeros((3, 1))
        self.aruco_noise_bias = np.zeros((3, 1))


        self.X = np.array([0.0 , 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        self.P = np.zeros((15,15))

        # self.U = np.array([0.0 , 0, 0, 0 ])
        
        self.process_noise = np.zeros((1, 15))
        self.Q = np.zeros((15,15))

        self.drone = QuadrotorDynamics()

        self.imu_H = np.array([
            [ 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ],
            [ 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ],
            [ 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ]
            ])

        self.aruco_H = np.array([
            [ 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ],
            [ 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ],
            [ 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ]
            ])
        

    def apply_system_dynamics(self, U, dt):
        A = self.drone.getA(self.X)        
        B = self.drone.getB(self.X)

        A_discretized = A * dt + np.eye(self.X.shape[0])
        B_discretized = B * dt
        

        # Predict State using system dynamics
        self.X = A_discretized @ self.X + B_discretized @ U + self.process_noise
        
        # Predict Process Noise Covariance
        self.P = A_discretized @ self.P @ A_discretized.T + self.Q
        return self.X, self.P



    def update_measurement(self, H, sensor_obs, sensor_noise_bias, sensor_noise_cov):
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
        return self.X, self.P


    def estimate_pose(self, control_input, sensor_obs, flag, dt):

        if flag == 0:
            # Both Empty
            self.apply_system_dynamics(control_input, dt)
            return self.X

        elif flag == 1:
            # Only IMU
            self.apply_system_dynamics(control_input, dt)
            self.update_measurement(self.imu_H, sensor_obs['imu'], self.imu_noise_bias, self.imu_noise_cov)
            return self.X

        elif flag == 2:
            # Only Aruco
            self.apply_system_dynamics(control_input, dt)
            self.update_measurement(self.aruco_H, sensor_obs['camera'], self.aruco_noise_bias, self.aruco_noise_cov)
            return self.X

        else:
            # Both
            self.apply_system_dynamics(control_input, dt)
            self.update_measurement(self.imu_H, sensor_obs['imu'], self.imu_noise_bias, self.imu_noise_cov)
            self.update_measurement(self.aruco_H, sensor_obs['camera'], self.aruco_noise_bias, self.aruco_noise_cov)
            return self.X
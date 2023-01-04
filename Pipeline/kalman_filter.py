import numpy as np

class KalmanFilter():

    # State X = [x, y, z, x', y', z', roll, pitch, yaw, roll', pitch', yaw']
    # Control Input U = [Upward Thurst (T - mg), Pitch Torque, Roll Torque, Yaw Torque]

    def __init__(self,
                 X=np.zeros((12, 1)),
                 P=np.zeros((12, 12)),
                 process_noise=np.zeros((12, 1)),
                 Q=np.zeros((12, 12)),
                 mass=0.1,
                 gravity=9.8,
                 MOI=(0.1, 0.1, 0.1)) -> None:
        
        # Estimates
        self.X = X
        self.P = P

        # Parameters
        self.mass = mass # in Kg
        self.gravity = gravity 
        self.MOI_x = MOI[0] # Moment of Inertia X
        self.MOI_y = MOI[1] # Moment of Inertia Y
        self.MOI_z = MOI[2] # Moment of Inertia Z

        self.process_noise = process_noise
        self.aruco_sensor_noise = np.zeros((6, 1))
        self.imu_sensor_noise = np.zeros((6, 1))

        self.Q = Q # State Model Noise Covariance Matrix [Process Noise Uncertainity]
        
        self.H = np.array([
            [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
        ]) # Measurement matrix, maps state -> sensor measurement
        
        # self.arcuo_noise_covariance = np.array([
        #     [0.37204485, -0.0042922, 0.00638548],
        #     [-0.0042922, 0.18774207, -0.01803895],
        #     [ 0.00638548, -0.01803895, 0.13632]
        #     ])
        # self.imu_noise_covariance = np.array([
        #     [0.2980565,0.03990039,0.01681866],
        #     [0.03990039,0.13436595,0.01088073],
        #     [0.01681866,0.01088073,0.62214558]
        #     ])

        # System Model Parameters
        self.A = np.array([
            [0, 0, 0, 1.0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1.0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, -self.gravity, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, self.gravity,  0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            ])
        
        self.B = np.array([
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [1/self.mass, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 1/self.MOI_x, 0, 0],
            [0, 0, 1/self.MOI_y, 0],
            [0, 0, 0, 1/self.MOI_z]
            ])
        

    def apply_system_dynamics(self, U, dt):
        # Predict State using system dynamics
        self.X = self.X + (self.A @ self.X + self.B @ U) * dt + self.process_noise
        # Predict Process Noise Covariance
        self.P = (self.A * dt + np.eye(self.X.shape[0])) @ self.P @ (self.A * dt + np.eye(self.X.shape[0])).T + self.Q



    def update_measurement(self, sensor_obs, sensor_noise_bias, sensor_noise_cov):
        # Calculate the measurement residual
        measurement_residual = sensor_obs - ((self.H @ self.X) + (sensor_noise_bias))                
        # Calculate the measurement residual covariance
        S = self.H @ self.P @ self.H.T + sensor_noise_cov

        # Calculate the near-optimal Kalman gain
        # We use pseudoinverse since some of the matrices might be non-square or singular.
        self.K = self.P @ self.H.T @ np.linalg.pinv(S)
            
        # Calculate an updated state estimate for time k
        self.X = self.X + (self.K @ measurement_residual)        
        # Update the state covariance estimate for time k
        M = self.K @ self.H
        self.P = (np.eye(M.shape[0]) - M) @ self.P @ (np.eye(M.shape[0]) - M).T + self.K @ sensor_noise_cov @ self.K.T #P_k - (K_k @ H_k @ P_k) 
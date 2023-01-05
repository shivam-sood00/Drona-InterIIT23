import numpy as np

class KalmanFilter():

    # State X = [x, y, z, x', y', z', roll, pitch, yaw, roll', pitch', yaw']
    # Control Input U = [Upward Thurst (T - mg), Pitch Torque, Roll Torque, Yaw Torque]

    def __init__(self) -> None:
        pass

    def apply_system_dynamics(self, X, A, B, U, Q, P, process_noise, dt):

        A_discretized = A * dt + np.eye(X.shape[0])
        B_discretized = B * dt
        

        # Predict State using system dynamics
        X = A_discretized @ X + B_discretized @ U + process_noise
        
        # Predict Process Noise Covariance
        P = A_discretized @ P @ A_discretized.T + Q
        return X,P



    def update_measurement(self, X, H, P, sensor_obs, sensor_noise_bias, sensor_noise_cov):
        # Calculate the measurement residual
        measurement_residual = sensor_obs - ((H @ X) + (sensor_noise_bias))                
        # Calculate the measurement residual covariance
        S = H @ P @ H.T + sensor_noise_cov

        # Calculate the near-optimal Kalman gain
        # We use pseudoinverse since some of the matrices might be non-square or singular.
        K = P @ H.T @ np.linalg.pinv(S)
            
        # Calculate an updated state estimate for time k
        X = X + (K @ measurement_residual)        
        # Update the state covariance estimate for time k
        M = K @ H
        P = (np.eye(M.shape[0]) - M) @ P @ (np.eye(M.shape[0]) - M).T + K @ sensor_noise_cov @ K.T #P_k - (K_k @ H_k @ P_k) 
        return X,P

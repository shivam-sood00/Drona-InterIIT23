import numpy as np
from vision.QuadrotorDynamics import QuadrotorDynamics

class KalmanFilter():

    # State X = [x, y, z, x', y', z', roll, pitch, yaw, roll', pitch', yaw']
    # Control Input U = [Upward Thurst (T - mg), Pitch Torque, Roll Torque, Yaw Torque]

    def __init__(self,debug = False) -> None:

        self.imu_noise_cov = np.array([[0.29801201,0.0260134,0.03899557],
                              [0.0260134,0.1585327,0.03870672],
                              [0.03899557,0.03870672,0.61070139]])

        self.aruco_noise_cov = np.array([[ 0.5834703,0.00638922,-0.06083379],
                              [ 0.00638922,0.31987258,-0.02961871],
                              [-0.06083379,-0.02961871,0.64220555]]) 


        self.imu_noise_bias = np.zeros(3)
        self.aruco_noise_bias = np.zeros(3)

        self.debug = debug

        self.X = np.array([0.0 , 0, 0, 0, 0, 0],dtype=np.float32)
        self.P = np.zeros((6,6),dtype=np.float32)
        
        self.process_noise = np.zeros(6)
        self.Q = np.eye(6)

        self.drone = QuadrotorDynamics()

        self.imu_H = np.array([
            [ 0, 0, 0, 1.0, 0, 0],
            [ 0, 0, 0, 0, 1, 0 ],
            [ 0, 0, 0, 0, 0, 1 ]],dtype=np.float32)

        self.aruco_H = np.array([
            [ 1.0, 0, 0, 0, 0, 0 ],
            [ 0, 1, 0, 0, 0, 0 ],
            [ 0, 0, 1, 0, 0, 0 ]],dtype=np.float32)
        

    def apply_system_dynamics(self,imuData, U, dt):
        A = self.drone.getA()        
        B = self.drone.getB(imuData)
        C = self.drone.getC(imuData)

        A_discretized = A * dt + np.eye(self.X.shape[0])
        B_discretized = B * dt
        C_discretized = C * dt

        # Predict State using system dynamics
        # print(A_discretized,self.X,B_discretized,self.process_noise)
        self.X = A_discretized @ self.X + B_discretized @ U + C_discretized + self.process_noise 
        
        # Predict Process Noise Covariance
        self.P = A_discretized @ self.P @ A_discretized.T + self.Q



    def update_measurement(self, H, sensor_obs, sensor_noise_bias, sensor_noise_cov):
        # Calculate the measurement residual
        #sensor_obs = sensor_obs.reshape((3, 1))
        measurement_residual = sensor_obs - ((H @ self.X) + (sensor_noise_bias))    

                    
        # Calculate the measurement residual covariance
        S = H @ self.P @ H.T + sensor_noise_cov

        #print(H.shape,self.P.shape,sensor_noise_cov.shape,S.shape)
        #(3, 15) (15, 15) (3, 3) (3, 3)

        # Calculate the near-optimal Kalman gain
        # We use pseudoinverse since some of the matrices might be non-square or singular.
        K = self.P @ H.T @ np.linalg.pinv(S)
            
        # Calculate an updated state estimate for time k
        self.X = self.X + (K @ measurement_residual)        
        # Update the state covariance estimate for time k
        M = K @ H
        self.P = (np.eye(M.shape[0]) - M) @ self.P @ (np.eye(M.shape[0]) - M).T + K @ sensor_noise_cov @ K.T #P_k - (K_k @ H_k @ P_k) 


    # def getRPY(self,sensorObs):
        
    #     imu_data = []
    #     imu_data.append(sensorObs["Roll"])  
    #     imu_data.append(sensorObs["Pitch"]) 
    #     imu_data.append(sensorObs["Yaw"])
        
    #     return np.array(imu_data)
    
    # def getXYZ(self,sensorObs):
    #     camera_data = np.zeros(3)
    #     camera_data[0] = sensorObs[1][0]/100
    #     camera_data[1] = sensorObs[1][1]/100
    #     camera_data[2] = sensorObs[2]
        
    #     return np.array(camera_data)
    
    def debugPrint(self,imu_data,cam_data):
        print("IMU Data : ",imu_data)
        print("Cam Data : ",cam_data)


    def estimate_pose(self, sensor_obs, dt):
        imuData = sensor_obs["imu"]
        cameraData = sensor_obs["cam"]
  
        # if flag == 3:
        #     # Both Empty
        #     self.apply_system_dynamics(control_input, dt)
        #     x = self.X

        # elif flag == 1:
        #     # Only IMU
        #     imu_data = self.getRPY(sensor_obs["imu"])
        #     self.apply_system_dynamics(control_input, dt)
        #     self.update_measurement(self.imu_H, imu_data, self.imu_noise_bias, self.imu_noise_cov)
        #     x = self.X

        # elif flag == 2:
        #     # Only Aruco
        #     camera_data = self.getXYZ(sensor_obs["camera"])
        #     self.apply_system_dynamics(control_input, dt)
        #     self.update_measurement(self.aruco_H, camera_data, self.aruco_noise_bias, self.aruco_noise_cov)
        #     x = self.X

        # else:
        #     # Both
        #     imu_data = self.getRPY(sensor_obs["imu"])
        #     camera_data = self.getXYZ(sensor_obs["camera"])

        self.apply_system_dynamics(imuData, imuData["AccZ"], dt)
        #self.update_measurement(self.imu_H, imuData, self.imu_noise_bias, self.imu_noise_cov)
        self.update_measurement(self.aruco_H, cameraData, self.aruco_noise_bias, self.aruco_noise_cov)
        

        # if self.debug:
        #     self.debugPrint(imu_data,camera_data)

        return self.X  
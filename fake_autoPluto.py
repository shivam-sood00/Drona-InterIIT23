from MSP_comms.plutoComms import COMMS
import threading
from approximatetimesync import time_sync
from vision.kalman_filter_v2 import KalmanFilter
from vision.fake_vision_pipeline2 import VisionPipeline
from vision.integrator import get_velocity,get_angle_rate
import time
import numpy as np
# from controls.pid_pluto import PID
from configparser import ConfigParser
import signal
import cv2

class autoPluto:
    def __init__(self,debug = False):
        signal.signal(signal.SIGINT, self.handler)
        self.debug = debug      
        self.config = ConfigParser()
        self.config.read('controls/fake_droneData.ini')  
        self.runLoopWaitTime = 0.04
        self.CamQueue = []
        self.currentState = None
        # self.action = {"Roll":1500,"Pitch":1500,"Yaw":1500,"Throttle":1500}
        self.mode =  self.config.get("Mode","mode")
        self.hover_reached_flag = True
        if self.mode == 'Rectangle':
            self.rectangle = [float(i) for i in self.config.get("Rectangle","xyz").split(',')]
        else:
            self.hover_z = float(self.config.get("Hover","z"))
        self.trajectory = []
        self.outOfBound = 0
        # print(self.config.sections())
        droneNumber = self.config.getint("Drone Number","droneNumber")
        # print(droneNumber)
        self.droneNo = self.config.sections()[droneNumber]
        ##########
        self.horizon  = 5
        self.data_fr_ma = np.zeros((3,self.horizon))
        self.counter = 0
        ##########
        # self.imuLock = threading.Lock()
        # self.commandLock = threading.sLock()
        z = int(self.config.get(self.droneNo,"id"))
        self.camera = VisionPipeline(rgb_res=(1080,1920),marker_size=3.6,required_marker_id=z,debug=1,padding = 0, imu_calib_data=[-0.03358463, 0.0135802, 0.0])
        self.lastTime = time.time()

        self.current_waypoint = np.array([0.,0.,0.]).reshape(3,1)

        self.xy_thresh = float(self.config.get("Thresholds","xy"))
        self.vel_thresh = float(self.config.get("Thresholds","vel"))
        self.z_thresh = float(self.config.get("Thresholds","z"))

        self.prevState = None
    
    def run(self):
        ret = 0
        i = 0
        first=True
        while(ret==0):
            # print("runloop")
            start_time_camera = time.time()
            self.camera.cam_process(self.CamQueue)
            
            if self.debug:
                print(f"{time.time()-start_time_camera} s for camera")
    
            start_time_pid = time.time()
            self.updateState()
            if self.currentState is None:
                # print("here")
                continue
            if first:
                # if yawUpdateFlag:
                #     yawUpdateFlag = False
                if self.mode == 'Rectangle':
                    self.trajectory.append([self.currentState[0],  self.currentState[1],  self.rectangle[2]])
                    self.trajectory.append([self.currentState[0]+self.rectangle[0],  self.currentState[1],  self.rectangle[2]])
                    self.trajectory.append([self.currentState[0]+self.rectangle[0],  self.currentState[1]+self.rectangle[1],  self.rectangle[2]])
                    self.trajectory.append([self.currentState[0],  self.currentState[1]+self.rectangle[1],  self.rectangle[2]])
                    self.trajectory.append([self.currentState[0],  self.currentState[1],  self.rectangle[2]])
                    self.axis_move = ['z','x','y','x','y']

                else:
                    self.trajectory.append([self.currentState[0],  self.currentState[1],  self.hover_z])
                    self.axis_move = ['z']

                
                self.current_waypoint = self.trajectory[i]
                self.camera.update_waypoint(self.trajectory[i])

                first = False
            
            err = np.array(self.current_waypoint) - np.array(self.currentState)
            print(self.currentState[0],self.currentState[1],self.currentState[2], err[0], err[1], err[2], self.current_waypoint[0], self.current_waypoint[1], self.current_waypoint[2])
            
            if self.prevState is not None and self.isReached_fake():
                if self.mode == "Rectangle": 
                    i += 1
                if i == len(self.trajectory) or self.mode !='Rectangle':
                    print("Reached Final Waypoint.")
                    
                    if self.mode =='Rectangle':
                        print("Now Landing")
                        break
                    else:
                        if self.hover_reached_flag:
                            print("Hovering")
                            self.hover_reached_flag = False
                else:
                    self.current_waypoint = self.trajectory[i]
                    self.camera.update_waypoint(self.trajectory[i])
                    print("IS Reached True")
            
            if self.debug:
                print(f"{time.time()-start_time_pid} s for pid")
            
            loop_time = time.time() - start_time_camera
            if loop_time < self.runLoopWaitTime:
                time.sleep(self.runLoopWaitTime - loop_time)
            
            if self.debug:
                print(f"Total time {loop_time}s")
            
        time.sleep(2)
    
    # update currentState
    def isReached_fake(self):
        """
        Signal when the drone is reached at its destination.
        """
        # if self.useWay:
        #     err = (self.target_pose[:2]-self.steady_state_err_way[:2]) - self.cur_pose[:2]
        # else:
        #     err = (self.target_pose[:2]-self.steady_state_err_hover[:2]) - self.cur_pose[:2]
        
        np_wp = np.array(self.current_waypoint).reshape(3,1)
        np_state = np.array(self.currentState).reshape(3,1)
        np_prev_state = np.array(self.prevState).reshape(3,1)

        err = np_wp - np_state

        # err = abs(self.cur_pose[:2] - (self.target_pose[:2]+self.))
        err = abs(err)
        # distCond = np.linalg.norm(err)
        velCond = np.linalg.norm(np_state - np_prev_state)/0.04
        self.vel_error = velCond
        print("x_err,y_err, velCond, z_err",err[0],err[1],velCond,err[2])
        if np.all(err[:2]< self.xy_thresh) and velCond < self.vel_thresh and err[2]<self.z_thresh:
            return True
        return False
    
    def updateState(self):

        self.prevState = self.currentState
        
        if len(self.CamQueue)>0:
            sensorData = self.CamQueue[-1] # current_time, aruco_pose, z_from_realsense

            for data in self.CamQueue:
                if(len(data)==1):
                    self.outOfBound = data
            self.CamQueue.clear()
            # print(sensorData)
            
            if self.outOfBound==0:
                if self.currentState is  None:
                    self.currentState = list(sensorData[1][:2,0]) + [sensorData[2]]
                else:
                    self.currentState[:3] = list(sensorData[1][:2,0]) + [sensorData[2]]      
        
        
        if self.currentState is not None:
            
            # Apply Moving Average on sensor data x y
            self.data_fr_ma[:,0:self.horizon-1] = self.data_fr_ma[:,1:self.horizon]
            self.data_fr_ma[0,self.horizon-1] = self.currentState[0]
            self.data_fr_ma[1,self.horizon-1] = self.currentState[1]  
            self.data_fr_ma[2,self.horizon-1] = self.currentState[2]   
            estimated = np.average(self.data_fr_ma, axis=1)     
            
            if self.counter < self.horizon:
                self.counter += 1
            else:
                self.currentState[0] = estimated[0]
                self.currentState[1] = estimated[1]
                self.currentState[2] = estimated[2]
        
    def handler(self, sigma, frame):
        msg = "Exit + Land"
        # self.comms.land()
        # self.comms.readLoop = False
        # self.comms.writeLoop = False
        # self.readThread.join()
        # self.writeThread.join()
        cv2.destroyAllWindows()
        print(msg)
        exit()


if __name__ == "__main__":
    drone1 = autoPluto()
    # print("arming")
    # drone1.comms.arm()
    # print("calling run")
    drone1.run()
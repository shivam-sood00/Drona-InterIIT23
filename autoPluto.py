from MSP_comms.plutoComms import COMMS
import threading
from approximatetimesync import time_sync
from vision.kalman_filter_v2 import KalmanFilter
from vision.vision_pipeline import VisionPipeline
from vision.integrator import get_angle_rate,get_velocity
import time
import numpy as np
# from controls.pid_pluto import PID
import pluto_pid_controller as PID
class autoPluto:
    def __init__(self,debug = False):
        self.comms = COMMS()
        self.debug = debug
        self.runLoopWaitTime = 0.04
        self.IMUQueue = []
        self.CamQueue = []
        self.currentState = None
        self.action = {"Roll":1500,"Pitch":1500,"Yaw":1500,"Throttle":1500}
        self.trajectory = [[0,0,0.9]]
        self.pid = PID
        self.outOfBound = 0
        ##########
        self.horizon  = 5
        self.data_fr_ma = np.zeros((9,self.horizon))
        self.counter = 0
        self.velocity = [0,0,0]
        self.angleRate = None
        ##########
        readThread = threading.Thread(target=self.comms.read,args=[self.IMUQueue])
        writeThread = threading.Thread(target=self.comms.write)
        cameraThread = threading.Thread(target=self.cameraFeed)
        writeThread.start()
        self.lastTime = time.time()
        readThread.start()
        cameraThread.start()
        self.first = True
        # print("started all threads")
        
    
    # updates queueXYZ
    def cameraFeed(self):
        camera = VisionPipeline(rgb_res=(1080,1920),marker_size=3.6,required_marker_id=1,debug=0)
        # camera = VisionPipeline(rgb_res=(1080,1920),marker_size=3.6,required_marker_id=6,debug=0)
        camera.cam_init()
        camera.cam_process(self.CamQueue)
    
# 
    def run(self):
        for point in self.trajectory:
            # print(point)
            # self.pid.set_target_pose(point=point)
            while(True):
                # print("runloop")
                self.updateState()
                if self.currentState is None:
                    continue
                if self.first:
                    self.pid.des_state.euler_angle[2] = self.currentState[-1]
                    self.first = False
                self.updateAction()
                self.takeAction()
                print(self.currentState[0],self.currentState[1],self.currentState[2],self.comms.paramsSet["Roll"],self.comms.paramsSet["Pitch"],self.comms.paramsSet["Yaw"],self.comms.paramsSet["Throttle"],self.pid.err_roll[0],self.pid.err_pitch[0],self.pid.err_thrust[0],self.currentState[3])
                time.sleep(self.runLoopWaitTime)
    
    # update currentState
    def updateState(self):
        # flag, sensorData = time_sync(self.IMUQueue,self.CamQueue)
        
        # EKF = KalmanFilter(debug=False)
        # currentTime = time.time()
        # self.currentState = EKF.estimate_pose(self.action,sensorData,flag,dt =currentTime-self.lastTime)
        # self.lastTime = currentTime
        
        if len(self.CamQueue)>0:
            sensorData = self.CamQueue[-1]
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
                    
            # self.currentState[2] = 2.8 -self.currentState[2]
        # if len(self.IMUQueue)>0:
        #     print("Pitch: ",self.IMUQueue[-1]["Pitch"])
        currentTime = time.time()
        dt = currentTime - self.lastTime
        if len(self.IMUQueue)>0:
            
            self.velocity = get_velocity(self.velocity,self.IMUQueue[-1],dt)
            self.angleRate = get_angle_rate(self.IMUQueue[-1],dt)

            if self.currentState is None:
                pass
            elif len(self.currentState)==3:
                self.currentState +=  [self.IMUQueue[-1]["Roll"],self.IMUQueue[-1]["Pitch"],self.IMUQueue[-1]["Yaw"]] + self.velocity + self.angleRate 
            else:
                self.currentState[3:] = [self.IMUQueue[-1]["Roll"],self.IMUQueue[-1]["Pitch"],self.IMUQueue[-1]["Yaw"]] + self.velocity + self.angleRate
            self.IMUQueue.clear()

        elif self.currentState is None:
            pass        
        
        elif len(self.currentState) == 3:
            self.currentState = None
        
        if self.currentState is not None:
            
            # Apply Moving Average on sensor data x y
            self.data_fr_ma[:,0:self.horizon-1] = self.data_fr_ma[:,1:self.horizon]

            for i in range(3):
                self.data_fr_ma[i,self.horizon-1] = self.currentState[i]
            
            estimated = np.average(self.data_fr_ma, axis=1)     
            
            if self.counter < self.horizon:
                self.counter += 1
            else:
                for i in range(3):
                    self.currentState[i] = estimated[i]
        
        self.lastTime = currentTime
        # if self.debug:
        # print("updated state: ",self.currentState)
    
    # update action
    def updateAction(self):
        
        # if len(self.IMUQueue)!=0:
        #     temp = self.IMUQueue[-1].copy()
        #     self.action[0] = temp["rcRoll"]
        #     self.action[1] = temp["rcPitch"]
        #     self.action[2] = temp["rcYaw"]
        #     self.action[3] = temp["rcThrottle"]   
        
        self.pid.odometry_callback(self.currentState)
        self.pid.main()
        # self.pid.update_pos(self.currentState)
        # self.pid.calc_err()
        # self.action["Pitch"] = self.pid.set_pitch()
        # self.action["Roll"] = self.pid.set_roll()

        self.action["Pitch"], self.action['Roll'] = self.pid.set_pitch_and_roll()
        self.action["Throttle"] = self.pid.set_thrust()
        self.action["Yaw"] = self.pid.set_yaw()
        # print("action: ",self.action)
    
    def takeAction(self):
        if self.outOfBound==0:
            # print("sending action")
            # converting to integer as we can only send integral values via MSP Packets
            self.comms.paramsSet["Roll"] = int(self.action["Roll"])
            self.comms.paramsSet["Pitch"] = int(self.action["Pitch"])
            self.comms.paramsSet["Throttle"] = int(self.action["Throttle"])
            self.comms.paramsSet["Yaw"] = int(self.action["Yaw"])
            # print("sent")
        else:
            self.comms.paramsSet["currentCommand"] = 2
            print("Landing: ",self.outOfBound)
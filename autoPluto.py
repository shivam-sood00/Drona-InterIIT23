from MSP_comms.plutoComms import COMMS
import threading
from approximatetimesync import time_sync
from vision.kalman_filter_v2 import KalmanFilter
from vision.vision_pipeline import VisionPipeline
import time
import numpy as np
from controls.pid_pluto import PID
class autoPluto:
    def __init__(self,debug = False):
        self.comms = COMMS()
        self.debug = debug
        self.runLoopWaitTime = 0.04
        self.IMUQueue = []
        self.CamQueue = []
        self.currentState = [0,0,0]
        self.action = {"Roll":1500,"Pitch":1500,"Yaw":1500,"Throttle":1500}
        self.trajectory = [[1,0,2],[-1,0,2]]
        self.pid = PID()
        self.outOfBound = 0
        readThread = threading.Thread(target=self.comms.read,args=[self.IMUQueue])
        writeThread = threading.Thread(target=self.comms.write)
        cameraThread = threading.Thread(target=self.cameraFeed)
        writeThread.start()
        self.lastTime = time.time()
        readThread.start()
        cameraThread.start()
        print("started all threads")
        
    
    # updates queueXYZ
    def cameraFeed(self):
        camera = VisionPipeline(rgb_res=(1080,1920),marker_size=3.6,required_marker_id=1,debug=0)
        camera.cam_init()
        camera.cam_process(self.CamQueue)
    
    # 
    def run(self):
        for point in self.trajectory:
            print(point)
            self.pid.set_target_pose(point=point)
            while(True):
                # print("runloop")
                self.updateState()
                self.updateAction()
                self.takeAction()
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
            # print(list(sensorData[1][:2,0]),[sensorData[2]])
            self.currentState = list(sensorData[1][:2,0]) + [sensorData[2]]
        self.IMUQueue.clear()

        # if self.debug:
        print("updated state: ",self.currentState)
    
    # update action
    def updateAction(self):
        
        # if len(self.IMUQueue)!=0:
        #     temp = self.IMUQueue[-1].copy()
        #     self.action[0] = temp["rcRoll"]
        #     self.action[1] = temp["rcPitch"]
        #     self.action[2] = temp["rcYaw"]
        #     self.action[3] = temp["rcThrottle"]   
        
        self.pid.update_pos(self.currentState)
        self.pid.calc_err()
        self.action["Pitch"] = self.pid.set_pitch()
        self.action["Roll"] = self.pid.set_roll()
        self.action["Throttle"] = self.pid.set_thrust()
        print("action: ",self.action)
    
    def takeAction(self):
        if self.outOfBound==0:
            self.comms.paramsSet["Roll"] = self.action["Roll"]
            self.comms.paramsSet["Pitch"] = self.action["Pitch"]
            self.comms.paramsSet["Throttle"] = self.action["Throttle"]
        else:
            self.comms.paramsSet["currentCommand"] = 2
            print("Landing: ",self.outOfBound)
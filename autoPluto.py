from MSP_comms.plutoComms import COMMS
import threading
from approximatetimesync import time_sync
from vision.kalman_filter_v2 import KalmanFilter
from vision.vision_pipeline import VisionPipeline
import time
import numpy as np
class autoPluto:
    def __init__(self,debug = False):
        self.comms = COMMS()
        self.debug = debug
        self.runLoopWaitTime = 0.04
        self.IMUQueue = []
        self.CamQueue = []
        self.currentState = []
        self.action = np.array([0.0,0.0,0.0,0.0])
        readThread = threading.Thread(target=self.comms.read,args=[self.IMUQueue])
        writeThread = threading.Thread(target=self.comms.write)
        cameraThread = threading.Thread(target=self.cameraFeed)
        writeThread.start()
        self.lastTime = time.time()
        readThread.start()
        cameraThread.start()
        
    
    # updates queueXYZ
    def cameraFeed(self):
        camera = VisionPipeline(rgb_res=(1080,1920),marker_size=3.6,required_marker_id=1)
        camera.cam_init()
        camera.cam_process(self.CamQueue)
        
    
    # 
    def run(self):
        while(True):
            if self.debug:
                print("IMU Queue:",self.IMUQueue)
                print("Cam Queue:",self.CamQueue)
            
            self.updateState()
            self.updateAction()
            # self.takeAction()
            
            time.sleep(self.runLoopWaitTime)
    
    # update currentState
    def updateState(self):
        flag, sensorData = time_sync(self.IMUQueue,self.CamQueue)
        EKF = KalmanFilter(debug=False)
        
        currentTime = time.time()
        self.currentState = EKF.estimate_pose(self.action,sensorData,flag,dt =currentTime-self.lastTime)
        self.lastTime = currentTime

        if self.debug:
            print("Flag:",flag,"Sensor Data:",sensorData)
    
    # update action
    def updateAction(self):
        if len(self.IMUQueue)!=0:
            temp = self.IMUQueue[-1].copy()
            self.action[0] = temp["rcRoll"]
            self.action[1] = temp["rcPitch"]
            self.action[2] = temp["rcYaw"]
            self.action[3] = temp["rcThrottle"]   
        #print(self.action)
    
    def takeAction(self):
        pass
        # self.comms[""]

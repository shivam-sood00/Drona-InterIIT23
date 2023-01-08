from MSP_comms.plutoComms import COMMS
import threading
from approximatetimesync import time_sync
from vision.kalman_filter_v2 import KalmanFilter


class autoPluto:
    def __init__(self):
        self.comms = COMMS()
        self.IMUQueue = []
        self.CamQueue = []
        self.currentState = []
        self.action = []
        readThread = threading.Thread(target=self.comms.read,args=[self.IMUQueue])
        writeThread = threading.Thread(target=self.comms.write)
        cameraThread = threading.Thread(target=self.cameraFeed)
        writeThread.start()
        readThread.start()
        cameraThread.start()
    
    # updates queueXYZ
    def cameraFeed(self):
        pass
    
    # 
    def run(self):
        while(True):
            print(self.IMUQueue)
            self.updateState()
            # self.updateAction()
            # self.takeAction()
    
    # update currentState
    def updateState(self):
        flag, sensorData = time_sync(self.IMUQueue,self.CamQueue)
        EKF = KalmanFilter()
        self.currentState = EKF.estimate_pose(self.action,sensorData,flag)
        pass
    
    # update action
    def decideAction(self):
        pass
    
    def takeAction(self):
        # self.comms[""]
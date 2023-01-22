from MSP_comms.plutoComms import COMMS
import threading
from approximatetimesync import time_sync
from vision.kalman_filter_v2 import KalmanFilter
from vision.vision_pipeline import VisionPipeline
from vision.movingAverage import movingAverage
import time
import numpy as np
from controls.pid_pluto import PID
from configparser import ConfigParser



class autoPluto:
    def __init__(self,debug = False):
        self.comms = COMMS()
        self.debug = debug        
        self.runLoopWaitTime = 0.04
        self.IMUQueue = []
        self.CamQueue = []
        self.currentState = {"lastVisionUpdate":-1,"lastImuUpdate":-1,"x":-1,"y":-1,"z":-1,"Roll":-1,"Pitch":-1,"Yaw":-1}
        self.action = {"Roll":1500,"Pitch":1500,"Yaw":1500,"Throttle":1500}
        self.trajectory = [[0,0,0.9]]
        # self.trajectory = [[0,0,0.9],[0.5,0,0],[0,-0.3,0],[-0.5,0,0]]
        self.outOfBound = 0
        self.config = ConfigParser()
        # if self.debug:
        self.config.read('controls/droneData.ini')
        # print(self.config.sections())
        droneNumber = self.config.getint("Drone Number","droneNumber")
        # print(droneNumber)
        self.droneNo = self.config.sections()[droneNumber]
        self.pid = PID(config=self.config,droneNo=self.droneNo)
        ##########
        self.w_dot__movingAverage = movingAverage(1,5)
        self.EKF = KalmanFilter()
        ##########
        self.comms.paramsSet["trimRoll"] = self.config.getint(self.droneNo,"trimRoll")
        self.comms.paramsSet["trimPitch"] = self.config.getint(self.droneNo,"trimPitch")
        self.visionTime = None
        self.controlTime = None
        self.imuTime = None
        # print(self.comms.paramsSet)
        # self.file = open('debug.csv', 'a+', newline ='')
        # with self.file:
        #     self.write = csv.writer(self.file)
        readThread = threading.Thread(target=self.comms.read,args=[self.IMUQueue])
        writeThread = threading.Thread(target=self.comms.write)
        cameraThread = threading.Thread(target=self.cameraFeed)
        writeThread.start()
        self.lastTime = time.time()
        readThread.start()
        cameraThread.start()
    
    # updates queueXYZ
    def cameraFeed(self):
        z = int(self.config.get(self.droneNo,"id"))
        # print(z)
        camera = VisionPipeline(rgb_res=(1080,1920),marker_size=3.6,required_marker_id=z,debug=0)
        camera.cam_init()
        camera.cam_process(self.CamQueue)
    
# 
    def run(self):
        ret = 0
        i = 0
        first=True
        yawUpdateFlag = True
        while(ret==0):
            # print("runloop")
            self.updateState()
            if self.currentState["lastVisionUpdate"]==-1 or self.currentState["lastImuUpdate"]==-1:
                continue
            if first:
                if i>=len(self.trajectory):
                    print("done..!!")
                    self.outOfBound = 3
                else:
                    point = self.trajectory[i]
                point[0] += self.currentState['x']
                point[1] += self.currentState['y']
                point[2] += self.currentState['z']
                i+=1
                if i!=1:
                    self.pid.useWay=True
                self.pid.set_target_pose(point=point)
                # print(self.pid.target_pose)
                if yawUpdateFlag:
                    yawUpdateFlag = False
                    self.pid.zero_yaw = self.currentState['Yaw']
                first = False
            print(self.currentState)
            self.updateAction()
            ret = self.takeAction()
            # data = [self.currentState[0],self.currentState[1],self.currentState[2],self.comms.paramsSet["Roll"],self.comms.paramsSet["Pitch"],self.comms.paramsSet["Yaw"],self.comms.paramsSet["Throttle"],self.pid.err_roll[0],self.pid.err_pitch[0],self.pid.err_thrust[0],self.currentState[3]]
            # if self.file:
            #     self.write.writerows(np.array(data,dtype=np.float64))
            # print("ok")
            print(self.currentState['x'],self.currentState['y'],self.currentState['z'],self.comms.paramsSet["Roll"],self.comms.paramsSet["Pitch"],self.comms.paramsSet["Yaw"],self.comms.paramsSet["Throttle"],self.pid.err_roll[0],self.pid.err_pitch[0],self.pid.err_thrust[0],self.currentState['Yaw'],self.currentState['Roll'],self.currentState['Pitch'])
            # print("not")
            time.sleep(self.runLoopWaitTime)
            # TODO: update target wavePoint when previous target reached
            if self.pid.isReached():
                first = True
                print("IS Reached True")
                # break
            
        time.sleep(2)
    
    # update currentState
    def updateState(self):
        # flag, sensorData = time_sync(self.IMUQueue,self.CamQueue)
        currentTime = time.time()
        if len(self.CamQueue)>0:
            sensorData = self.CamQueue[-1] # current_time, aruco_pose, z_from_realsense
            
            bias = 0
            sensorData["imu"]["AccZ"] -= bias
            sensorData["imu"]["AccZ"] = self.w_dot__movingAverage.getAverage(sensorData["imu"]["AccZ"])[0]

            for data in self.CamQueue:
                if(len(data)==1):
                    self.outOfBound = data
            self.CamQueue.clear()
            # print(sensorData)
            
            if self.outOfBound==0:
                    self.currentState['x'] = sensorData[1][0,0]
                    self.currentState['y'] = sensorData[1][1,0]
                    self.currentState['z'] = sensorData[2]
                    self.currentState["lastVisionUpdate"] = sensorData[0]
                    #self.currentState[:3] = list(sensorData[1][:2,0]) + [sensorData[2]]
                    
            # self.currentState[2] = 2.8 -self.currentState[2]
        # if len(self.IMUQueue)>0:
        #     print("Pitch: ",self.IMUQueue[-1]["Pitch"])
        if len(self.IMUQueue)>0:
            if self.currentState is None:
                pass
            # elif len(self.currentState)==3:
            #     self.currentState["Roll"] =  self.IMUQueue[-1]["Yaw"]
            #     self.currentState["Pitch"] = self.IMUQueue[-1]["Roll"]
            #     self.currentState["Yaw"] = self.IMUQueue[-1]["Pitch"]
            else:
                self.currentState["Roll"] =  self.IMUQueue[-1]["Roll"]
                self.currentState["Pitch"] = self.IMUQueue[-1]["Pitch"]
                self.currentState["Yaw"] = self.IMUQueue[-1]["Yaw"]
                self.currentState["lastImuUpdate"] = self.IMUQueue[-1]["timeOfLastUpdate"]
            self.IMUQueue.clear() 
        
        
        if self.currentState["lastVisionUpdate"] != -1:    
            EstimatedStates = self.EKF.estimate_pose(self.currentState, sensorData, dt = currentTime - self.lastTime)
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
        
        self.pid.update_pos(self.currentState)
        self.pid.calc_err()
        # self.action["Pitch"] = self.pid.set_pitch()
        # self.action["Roll"] = self.pid.set_roll()

        self.action["Pitch"], self.action['Roll'] = self.pid.set_pitch_and_roll()
        self.action["Throttle"] = self.pid.set_thrust()
        self.action["Yaw"] = self.pid.set_yaw()
        self.controlTime = time.time()
        # print("action: ",self.action)
    
    def takeAction(self):
        if self.outOfBound==0:
            # print("sending action")
            # converting to integer as we can only send integral values via MSP Packets
            # print("imu latency: ",self.controlTime-self.currentState["lastImuUpdate"],"\tvision latency: ",self.controlTime-self.currentState["lastVisionUpdate"])
            self.comms.paramsSet["Roll"] = int(self.action["Roll"])
            self.comms.paramsSet["Pitch"] = int(self.action["Pitch"])
            self.comms.paramsSet["Throttle"] = int(self.action["Throttle"])
            self.comms.paramsSet["Yaw"] = int(self.action["Yaw"])
            # print("sent")
            return 0
        else:
            self.comms.paramsSet["currentCommand"] = 2
            print("Landing: ",self.outOfBound)
            return 1
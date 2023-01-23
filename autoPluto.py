from MSP_comms.plutoComms import COMMS
import threading
from approximatetimesync import time_sync
from vision.kalman_filter_v2 import KalmanFilter
from vision.vision_pipeline import VisionPipeline
from vision.integrator import get_velocity,get_angle_rate
import time
import numpy as np
from controls.pid_pluto import PID
from configparser import ConfigParser
import signal


class autoPluto:
    def __init__(self,debug = False):
        signal.signal(signal.SIGINT, self.handler)
        self.comms = COMMS()
        self.debug = debug      
        self.config = ConfigParser()
        # if self.debug:
        self.config.read('controls/droneData.ini')  
        self.runLoopWaitTime = 0.04
        self.IMUQueue = []
        self.CamQueue = []
        self.currentState = None
        self.action = {"Roll":1500,"Pitch":1500,"Yaw":1500,"Throttle":1500}
        self.rectangle = [float(i) for i in self.config.get("Rectangle","xyz").split(',')]
        self.trajectory = []
        self.outOfBound = 0
        # print(self.config.sections())
        droneNumber = self.config.getint("Drone Number","droneNumber")
        # print(droneNumber)
        self.droneNo = self.config.sections()[droneNumber]
        self.pid = PID(config=self.config,droneNo=self.droneNo)
        ##########
        self.horizon  = 5
        self.data_fr_ma = np.zeros((3,self.horizon))
        self.counter = 0
        ##########
        self.comms.paramsSet["trimRoll"] = self.config.getint(self.droneNo,"trimRoll")
        self.comms.paramsSet["trimPitch"] = self.config.getint(self.droneNo,"trimPitch")
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
        camera = VisionPipeline(rgb_res=(1080,1920),marker_size=3.6,required_marker_id=z,debug=1,padding = 0)
        camera.cam_init()
        camera.cam_process(self.CamQueue)
    
    def run(self):
        ret = 0
        i = 0
        first=True
        # yawUpdateFlag = True
        while(ret==0):
            # print("runloop")
            self.updateState()
            if self.currentState is None:
                # print("here")
                continue
            if first:
                # if yawUpdateFlag:
                #     yawUpdateFlag = False
                self.pid.zero_yaw = self.currentState[3]
                self.trajectory.append([self.currentState[0],  self.currentState[1],  self.rectangle[2]])
                self.trajectory.append([self.currentState[0]+self.rectangle[0],  self.currentState[1],  self.rectangle[2]])
                self.trajectory.append([self.currentState[0]+self.rectangle[0],  self.currentState[1]+self.rectangle[1],  self.rectangle[2]])
                self.trajectory.append([self.currentState[0],  self.currentState[1]+self.rectangle[1],  self.rectangle[2]])
                self.trajectory.append([self.currentState[0],  self.currentState[1],  self.rectangle[2]])
                self.axis_move = ['z','x','y','x','y']
                self.pid.set_target_pose(self.trajectory[i],self.axis_move[i])
                
                # if i>=len(self.trajectory):
                #     print("done..!!")
                #     self.outOfBound = 3
                # else:
                #     point = self.trajectory[i]
                #     if i == 0:
                #         point[0] += self.currentState[0]
                #         point[1] += self.currentState[1]
                #         point[2] += self.currentState[2]
                #     # else:
                #     #     point = 
                # i+=1
                # if i!=1:
                #     self.pid.useWay = True
                # self.pid.set_target_pose(point=point)
                # print("Target: ")
                
                first = False
            self.updateAction()
            ret = self.takeAction()
            # data = [self.currentState[0],self.currentState[1],self.currentState[2],self.comms.paramsSet["Roll"],self.comms.paramsSet["Pitch"],self.comms.paramsSet["Yaw"],self.comms.paramsSet["Throttle"],self.pid.err_roll[0],self.pid.err_pitch[0],self.pid.err_thrust[0],self.currentState[3]]
            # if self.file:
            #     self.write.writerows(np.array(data,dtype=np.float64))
            # print("ok")
            print(self.currentState[0],self.currentState[1],self.currentState[2],self.comms.paramsSet["Roll"],self.comms.paramsSet["Pitch"],self.comms.paramsSet["Yaw"],self.comms.paramsSet["Throttle"],self.pid.err_roll[0],self.pid.err_pitch[0],self.pid.err_thrust[0],self.currentState[3],self.currentState[4],self.currentState[5],self.pid.err_pitch_with_sse[0],self.pid.err_roll_with_sse[0],self.pid.err_thrust_with_sse[0])
            # print("not")
            time.sleep(self.runLoopWaitTime)
            if self.pid.isReached():
                i += 1
                if i == len(self.trajectory):
                    print("Reached Final Waypoint. Now Landing")
                    self.outOfBound = 3
                else:
                    self.pid.useWay = True
                    self.pid.set_target_pose(self.trajectory[i],self.axis_move[i])
                    print("IS Reached True")
                
               
                # break
            
        time.sleep(2)
    
    # update currentState
    def updateState(self):
        # flag, sensorData = time_sync(self.IMUQueue,self.CamQueue)
        
        # EKF = KalmanFilter(debug=False)
        # currentTime = time.time()
        # self.currentState = EKF.estimate_pose(self.action,sensorData,flag,dt =currentTime-self.lastTime)
        
        # self.lastTime = currentTime
        
        
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
                    
            # self.currentState[2] = 2.8 -self.currentState[2]
        # if len(self.IMUQueue)>0:
        #     print("Pitch: ",self.IMUQueue[-1]["Pitch"])
        if len(self.IMUQueue)>0:
            if self.currentState is None:
                pass
            elif len(self.currentState)==3:
                self.currentState +=  [self.IMUQueue[-1]["Yaw"],self.IMUQueue[-1]["Roll"],self.IMUQueue[-1]["Pitch"]]
            else:
                self.currentState[-3:] =  [self.IMUQueue[-1]["Yaw"],self.IMUQueue[-1]["Roll"],self.IMUQueue[-1]["Pitch"]]
            self.IMUQueue.clear()

        elif self.currentState is None:
            pass        
        
        elif len(self.currentState) == 3:
            self.currentState = None
        
        
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
            return 0
        else:
            self.comms.paramsSet["currentCommand"] = 2
            print("Landing: ",self.outOfBound)
            return 1
        
    def handler(self, sigma, frame):
        msg = "Exit + Land"
        self.comms.land()
        print(msg)
        exit()
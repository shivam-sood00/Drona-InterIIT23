from task3.MSP_comms.plutoComms import COMMS
from task3.controls.pid_pluto import PID
import threading
import numpy as np


class autoPluto:
    def __init__(self,config,droneNo) :
        self.currentState = {"x":None,"y":None,"z":None}
        self.action =  {"Roll":1500,"Pitch":1500,"Yaw":1500,"Throttle":1500}
        self.config = config
        self.droneNo = droneNo
        
        self.comms = COMMS(IP= self.config.get(self.droneNo,"IP"),Port = self.config.getint(self.droneNo,"Port"))
        self.pid = PID(self.config,self.droneNo)

        self.target = None
        self.axis = None
        
        self.comms.paramsSet["trimRoll"] = self.config.getint(self.droneNo,"trimRoll")
        self.comms.paramsSet["trimPitch"] = self.config.getint(self.droneNo,"trimPitch")
        
        self.state_filter_thresh = float(self.config.get("State","thresh"))
        self.carrot_res = {}
        self.carrot_res['x'] = float(self.config.get("Carrot","res_x"))
        self.carrot_res['y'] = float(self.config.get("Carrot","res_y"))
        self.carrot_res['z'] = float(self.config.get("Carrot","res_z"))
        
        self.commandLock = threading.Lock()
        self.ImuLock = threading.Lock()
        self.readThread = threading.Thread(target=self.comms.read,args=[self.ImuLock])
        self.writeThread = threading.Thread(target=self.comms.write,args=[self.commandLock])
        self.writeThread.start()
        self.readThread.start()
    
    def updateState(self,xyz):
        if xyz is None:
            return
        if self.currentState['x'] is not None:
            if (abs(xyz[0] - self.currentState['x']) > self.state_filter_thresh) or (abs(xyz[1] - self.currentState['y']) > self.state_filter_thresh) or (abs(xyz[2] - self.currentState['z']) > self.state_filter_thresh):
                return
        
        self.currentState['x'] = xyz[0]
        self.currentState['y'] = xyz[1]
        self.currentState['z'] = xyz[2]
        """
        TODO: 
        1. Apply some kind of  filter here. Ex Moving Average or Exponential Weighted Average
        """
    
    def updateAction(self):
        curPose = []
        curPose.extend(self.currentState.values())
        self.ImuLock.acquire()
        curPose.append(self.comms.paramsReceived["Yaw"])
        curPose.append(self.comms.paramsReceived["Roll"])
        curPose.append(self.comms.paramsReceived["Pitch"])
        self.ImuLock.release()
        self.pid.update_pos(curPose=curPose)


        self.pid.set_target_pose(self.set_carrot_wps(self.target,self.currentState),self.axis)
        self.pid.calc_err()

        self.action["Pitch"], self.action['Roll'] = self.pid.set_pitch_and_roll()
        self.action["Throttle"] = self.pid.set_thrust()
        self.action["Yaw"] = self.pid.set_yaw()
    
    def takeAction(self,exception = 0):
        self.commandLock.acquire()
        if exception==0:
            # converting to integer as we can only send integral values via MSP Packets
            self.comms.paramsSet["Roll"] = int(self.action["Roll"])
            self.comms.paramsSet["Pitch"] = int(self.action["Pitch"])
            self.comms.paramsSet["Throttle"] = int(self.action["Throttle"])
            self.comms.paramsSet["Yaw"] = int(self.action["Yaw"])
        elif exception==100:
            if bool(self.config.get("Reset","active")):
                print("reseting Pid")

                roll_clip = 9
                pitch_clip = 9
                throttle_clip = 300
                yaw_clip = 9


                self.comms.paramsSet["Roll"] = np.clip(int(self.action["Roll"]),1500 - roll_clip, 1500 + roll_clip)
                self.comms.paramsSet["Pitch"] = np.clip(int(self.action["Pitch"]),1500 - pitch_clip, 1500 + pitch_clip)
                self.comms.paramsSet["Throttle"] = np.clip(int(self.action["Throttle"]),1525 - throttle_clip, 1525 + throttle_clip)
                self.comms.paramsSet["Yaw"] = np.clip(int(self.action["Yaw"]),1500 - yaw_clip, 1500 + yaw_clip)
        else:
            self.comms.paramsSet["currentCommand"] = 2
            print("landing due to exception: ",exception)
        self.commandLock.release()
    
    def updateTarget(self,target,axis):
        self.target = target
        self.axis = axis
        if axis=='z':
            self.ImuLock.acquire()
            self.pid.zero_yaw = self.comms.paramsReceived["Yaw"]
            self.ImuLock.release()
    
    def isReached(self):
        return self.pid.isReached()
    
    def arm(self):
        self.comms.arm()
    
    def closeThreads(self):
        self.comms.writeLoop = False
        self.comms.readLoop = False
        self.writeThread.join()
        self.readThread.join()
    
    def set_carrot_wp(self,target,cur,res):
        if target > cur:
            return min(cur+res,target) 
        else:
            return max(cur-res,target)

    def set_carrot_wps(self,target,cur):
        carrot_target = []
        for i,axes in enumerate(['x','y','z']):
            carrot_target.append(self.set_carrot_wp(target[i],cur[axes],self.carrot_res[axes]))
        return carrot_target
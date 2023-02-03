from MSP_comms.plutoComms import COMMS
from controls.pid_pluto import PID
import threading


class autoPluto:
    def __init__(self,config,droneNo) :
        self.currentState = {"x":None,"y":None,"z":None}
        self.action =  {"Roll":1500,"Pitch":1500,"Yaw":1500,"Throttle":1500}
        self.config = config
        self.droneNo = droneNo
        
        self.comms = COMMS(IP= self.config.get(self.droneNo,"IP"),Port = self.config.getint(self.droneNo,"Port"))
        self.pid = PID(self.config,self.droneNo)
        
        self.comms.paramsSet["trimRoll"] = self.config.getint(self.droneNo,"trimRoll")
        self.comms.paramsSet["trimPitch"] = self.config.getint(self.droneNo,"trimPitch")
        
        self.commandLock = threading.Lock()
        self.readThread = threading.Thread(target=self.comms.read)
        self.writeThread = threading.Thread(target=self.comms.write,args=[self.commandLock])
        self.writeThread.start()
        self.readThread.start()
    
    def updateState(self,xyz):
        print(xyz)
        self.currentState['x'] = xyz[0]
        self.currentState['y'] = xyz[1]
        self.currentState['z'] = xyz[2]
        """
        TODO: 
        1. Apply some kind of  filter here. Ex Moving Average or Exponential Weighted Average
        """
    
    def updateAcion(self):
        curPose = []
        curPose.extend(self.currentState.values())
        curPose.append(self.comms.paramsReceived["Yaw"])
        curPose.append(self.comms.paramsReceived["Roll"])
        curPose.append(self.comms.paramsReceived["Pitch"])
        self.pid.update_pos(curPose=curPose)

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
        else:
            self.comms.paramsSet["currentCommand"] = 2
            print("landing due to exception: ",exception)
        self.commandLock.release()
    
    def updateTarget(self,target,axis):
        self.pid.set_target_pose(target,axis)
        if axis=='z':
            self.pid.zero_yaw = self.comms.paramsReceived["Yaw"]
    
    def isReached(self):
        return self.pid.isReached()
    
    def arm(self):
        self.comms.arm()
    
    def closeThreads(self):
        self.comms.writeLoop = False
        self.comms.readLoop = False
        self.writeThread.join()
        self.readThread.join()
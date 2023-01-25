from MSP_comms.plutoComms import COMMS
import threading
from approximatetimesync import time_sync
from vision.kalman_filter_v2 import KalmanFilter
from vision.vision_pipeline2 import VisionPipeline
from vision.integrator import get_velocity,get_angle_rate
import time
import numpy as np
from controls.pid_pluto import PID
from configparser import ConfigParser
import signal
import cv2
import struct,math


def sign(s):
    if s==0:
        return 0
    elif s>0:
        return 1
    return -1

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
        self.pid = PID(config=self.config,droneNo=self.droneNo)
        ##########
        self.horizon  = 5
        self.data_fr_ma = np.zeros((3,self.horizon))
        self.counter = 0
        ##########
        self.comms.paramsSet["trimRoll"] = self.config.getint(self.droneNo,"trimRoll")
        self.comms.paramsSet["trimPitch"] = self.config.getint(self.droneNo,"trimPitch")
        self.imuLock = threading.Lock()
        self.commandLock = threading.Lock()
        # print(self.comms.paramsSet)
        # self.file = open('debug.csv', 'a+', newline ='')
        # with self.file:
        #     self.write = csv.writer(self.file)
        z = int(self.config.get(self.droneNo,"id"))
        self.camera = VisionPipeline(rgb_res=(1080,1920),marker_size=3.6,required_marker_id=z,debug=1,padding = 0, imu_calib_data=[-0.03358463, 0.0135802, 0.0])
        self.useThrottle = True
        self.readThread = threading.Thread(target=self.comms.read,args=[self.IMUQueue, self.imuLock])
        self.writeThread = threading.Thread(target=self.comms.write,args=[self.commandLock])
        self.rcThread = threading.Thread(target=self.rc)
        self.rcThread.start()
        self.writeThread.start()
        self.lastTime = time.time()
        self.readThread.start()

    def rc(self):
        infile_path = "/dev/input/js0"
        EVENT_SIZE = struct.calcsize("LhBB")
        file = open(infile_path, "rb")
        event = file.read(EVENT_SIZE)
        while event:
        # inputVal = input("enter key: ")
            try: 
                (tv_msec,  value, type, number) = struct.unpack("LhBB", event)
                if type==2:
                    if number==0:
                        if not self.useThrottle:
                            val = 200*sign(value)*math.log(1+abs(value)/2**15)/math.log(2)
                            # val = val/(math.cos(comms.paramsReceived["Roll"])*math.cos(comms.paramsReceived["Pitch"]))
                            self.comms.paramsSet["Throttle"] = 1650 + int(val)
                        else:
                            if value>30000:
                                self.useThrottle = False
                    if number==3:
                        self.comms.paramsSet["Yaw"] = 1500 + int( 350*sign(value)*math.log(1+abs(value)/2**15)/math.log(2))
                    if number==2:
                        self.comms.paramsSet["Pitch"] = 1500 + int( 150*sign(value)*math.log(1+abs(value)/2**15)/math.log(2))
                    if number==1:
                        self.comms.paramsSet["Roll"] = 1500 + int( 150*sign(value)*math.log(1+abs(value)/2**15)/math.log(2))
                        

                event = file.read(EVENT_SIZE)
            except KeyboardInterrupt:
                print("landing")
                self.comms.land()
                break
    
    def run(self):
        ret = 0
        i = 0
        first=True
        # yawUpdateFlag = True
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
                self.pid.zero_yaw = self.currentState[3]
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

                
                self.pid.set_target_pose(self.trajectory[i],self.axis_move[i])
                self.camera.update_waypoint(self.trajectory[i])
                
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
            self.commandLock.acquire()
            print(self.currentState[0],self.currentState[1],self.currentState[2],
                    self.comms.paramsSet["Roll"],self.comms.paramsSet["Pitch"],
                    self.comms.paramsSet["Yaw"],self.comms.paramsSet["Throttle"],
                    self.pid.err_roll[0],self.pid.err_pitch[0],self.pid.err_thrust[0],
                    self.currentState[3],self.currentState[4],self.currentState[5],
                    self.pid.err_pitch_with_sse[0],self.pid.err_roll_with_sse[0],
                    self.pid.err_thrust_with_sse[0],self.pid.err_pitch[1],self.pid.err_roll[1],
                    self.pid.err_thrust[1],self.pid.err_pitch[2],self.pid.err_roll[2],
                    self.pid.err_thrust[2],self.pid.vel_error)
            self.commandLock.release()
            # print("not")
            if self.pid.isReached():
                if self.mode == "Rectangle": 
                    i += 1
                if i == len(self.trajectory) or self.mode !='Rectangle':
                    print("Reached Final Waypoint.")
                    
                    if self.mode =='Rectangle':
                        print("Now Landing")
                        self.outOfBound = 3
                    else:
                        if self.hover_reached_flag:
                            print("Hovering")
                            self.hover_reached_flag = False
                else:
                    self.pid.useWay = True
                    self.pid.set_target_pose(self.trajectory[i],self.axis_move[i])
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
        self.imuLock.acquire()
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
        self.imuLock.release()
        
        
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
        self.commandLock.acquire()
        if self.outOfBound==0:
            # print("sending action")
            # converting to integer as we can only send integral values via MSP Packets
            # self.comms.paramsSet["Roll"] = int(self.action["Roll"])
            # self.comms.paramsSet["Pitch"] = int(self.action["Pitch"])
            if self.useThrottle:
                self.comms.paramsSet["Throttle"] = int(self.action["Throttle"])
            # self.comms.paramsSet["Yaw"] = int(self.action["Yaw"])
            self.commandLock.release()
            # print("sent")
            return 0
        else:
            self.comms.paramsSet["currentCommand"] = 2
            self.commandLock.release()
            print("Landing: ",self.outOfBound)
            return 1
        
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

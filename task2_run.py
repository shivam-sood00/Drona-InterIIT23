from task2.MSP_comms.plutoComms import COMMS
import threading
from task2.vision.vision_pipeline import VisionPipeline
import time
import numpy as np
from task2.controls.pid_pluto import PID
from configparser import ConfigParser
import signal
import cv2

class autoPluto:
    """
        This is a class integrating all the modules of the pipeline and sending
        the data to the drone using MSP Packets. We integrate the controls, vision and
        state estimation modules and it is an easy-to-use class.

        Attributes:
            DEBUG: flag to enable prints in code for debugging. Default is False.
    """
    def __init__(self,debug = False):
        """
            self.comms: Object of the class COMMS that handles all communication with the pluto drone.
            self.debug: To enable debugging.
            self.config: Object of ConfigParser used to parse data from configuration files (in this case, from droneData.ini).   
            
            self.runLoopWaitTime: Variable to control the time period of the run loop that updates the states and take action based on PID output.
            self.IMUQueue: Stores all the incoming IMU data in a queue and is updated by COMMS.read method.
            self.CamQueue: Stores all the incoming Aruco Position data in a queue and is updated using VisionPipeline.cam_process method.
            self.currentState: Stores the current state of the drone, obtained after filtering.
            self.action: Dictionary of the control inputs to be given to the drone.
            self.mode: Defined in the droneData.ini file, used to switch between rectangle mode and hover mode.
            
            self.rectangle: If rectangle mode enabled, generates waypoint in (x,y,z) of in a rectangular pathline, whose lengths, breadths and height are defined in droneData.ini.   
            self.hover_z: If hover mode enabled, get the z coordinate setpoint from the droneData.ini.
            self.trajectory: Depending on the mode, stores the data of self.rectangle or self.hover_z.

            self.outOfBound: Flag to trigger, if drone goes out of frame, and hence take necessary action.
            self.droneNo: Stores the drone no, assigned in droneData.ini.
            self.pid: Creates object of PID Controller to control the drone.
            
            self.horizon: Horizon to consider while performing moving average filter.
            self.data_fr_ma: Stores the recent n data for performing moving average filter.
            self.counter: Intial counter to keep track of readings in the initial n-1 timestep, to prevent wrong calculation of moving average filter.

            self.comms.paramsSet["trimRoll"]: Set the trim values for Roll.
            self.comms.paramsSet["trimPitch"]: Set the trim values for Pitch.
            
            self.imuLock: Two threads are using common variable to update and access the data, we lock the memory to prevent simulataneous reading and writing of the data in the shared variable.
            self.commandLock: Two threads are using common variable to update and access the data, we lock the memory to prevent simulataneous reading and writing of the data in the shared variable.
            
            self.camera: Creates a vision pipeline object, to initialize RGB and depth camera and process the video frames to extract necessary data.
            self.readThread: Creates a read thread to recieve and store data coming from the drone, such as IMU Data.
            self.writeThread: Creates a write thread to send MSP commands to the drone.
            
            self.writeThread.start(): To start the writing thread.
            self.lastTime = time.time(): Stores the last time (used to calculate time elapsed).
            self.readThread.start(): To start the reading thread.
        """
        signal.signal(signal.SIGINT, self.handler)
        self.comms = COMMS()
        self.debug = debug      
        self.config = ConfigParser()
        self.config.read('task2/controls/droneData.ini')  
        self.runLoopWaitTime = 0.04
        self.IMUQueue = []
        self.CamQueue = []
        self.axis_move = []
        self.currentState = None
        self.action = {"Roll":1500,"Pitch":1500,"Yaw":1500,"Throttle":1500}
        self.mode =  self.config.get("Mode","mode")
        self.res_x = 2
        self.res_y = 2
        self.hover_reached_flag = True
        if self.mode == 'Rectangle':
            self.rectangle = [float(i) for i in self.config.get("Rectangle","xyz").split(',')]
        else:
            self.hover_z = float(self.config.get("Hover","z"))
        self.trajectory = []
        self.outOfBound = 0
        self.start_traversal = 0
        self.end_traversal = 0
        droneNumber = self.config.getint("Drone Number","droneNumber")
        self.droneNo = self.config.sections()[droneNumber]
        self.pid = PID(config=self.config,droneNo=self.droneNo)
        ##########
        self.horizon  = self.config.getint("Horizon","moving_horizon")
        self.data_fr_ma = np.zeros((3,self.horizon))
        self.counter = 0
        ##########
        self.comms.paramsSet["trimRoll"] = self.config.getint(self.droneNo,"trimRoll")
        self.comms.paramsSet["trimPitch"] = self.config.getint(self.droneNo,"trimPitch")
        self.imuLock = threading.Lock()
        self.commandLock = threading.Lock()
        z = int(self.config.get(self.droneNo,"id"))
        self.camera = VisionPipeline(rgb_res=(1080,1920),marker_size=3.6,required_marker_id=z,debug=1,padding = 0)
        
        self.readThread = threading.Thread(target=self.comms.read,args=[self.IMUQueue, self.imuLock])
        self.writeThread = threading.Thread(target=self.comms.write,args=[self.commandLock])
        self.writeThread.start()
        self.lastTime = time.time()
        self.readThread.start()
        self.carrot_res = {}
        self.carrot_res['x'] = float(self.config.get("Carrot","res_x"))
        self.carrot_res['y'] = float(self.config.get("Carrot","res_y"))
        self.carrot_res['z'] = float(self.config.get("Carrot","res_z"))
        self.state_filter_thresh = float(self.config.get("State","thresh"))

    
    def set_carrot_wp(self,target,cur,res):
        if target > cur:
            return min(cur+res,target) 
        else:
            return max(cur-res,target)

    def set_carrot_wps(self,target,cur):
        carrot_target = []
        for i,axes in enumerate(['x','y','z']):
            carrot_target.append(self.set_carrot_wp(target[i],cur[i],self.carrot_res[axes]))
        return carrot_target
        
    
    def run(self):
        """
        Implements the complete pipeline on the drone integrating the PID controller with pose estimation using
        the vision pipeline. Flags are setup to update the current condition of the drone, i.e., hovering, landing, etc.
        """
        ret = 0
        i = 0
        first=True
        while(ret==0):
            start_time_camera = time.time()
            self.camera.cam_process(self.CamQueue)
            
            if self.debug:
                pass
                # print(f"{time.time()-start_time_camera} s for camera")
    
            start_time_pid = time.time()
            self.updateState()
            if self.currentState is None:
                continue
            if first:
                self.pid.zero_yaw = self.currentState[3]
                if self.mode == 'Rectangle':
                    self.trajectory.append([self.currentState[0],  self.currentState[1],  self.currentState[2]+self.rectangle[2]])
                    self.axis_move.append('z')
                    self.res_x = int(self.res_x)
                    self.res_y = int(self.res_y)
                    for j in range(1,self.res_x):
                        self.trajectory.append([self.currentState[0]+j*self.rectangle[0]/(self.res_x-1),  self.currentState[1],  self.currentState[2]+self.rectangle[2]])
                        self.axis_move.append('x')
                    for j in range(1,self.res_y):
                        self.trajectory.append([self.currentState[0]+self.rectangle[0],  self.currentState[1]+j*self.rectangle[1]/(self.res_y-1),  self.currentState[2]+self.rectangle[2]])
                        self.axis_move.append('y')
                    for j in range(1,self.res_x):
                        self.trajectory.append([self.currentState[0]+(self.res_y-1-j)*self.rectangle[0]/(self.res_x-1),  self.currentState[1]+self.rectangle[1],  self.currentState[2]+self.rectangle[2]])
                        self.axis_move.append('x')
                    for j in range(1,self.res_y):
                        self.trajectory.append([self.currentState[0],  self.currentState[1]+(self.res_y-1-j)*self.rectangle[1]/(self.res_y-1),  self.currentState[2]+self.rectangle[2]])
                        self.axis_move.append('y')
                    # self.axis_move = ['z','x','y','x','y']
                    # print("Trajectory Waypoints:")
                    # print(self.trajectory)

                else:
                    self.trajectory.append([self.currentState[0],  self.currentState[1],  self.currentState[2]+self.hover_z])
                    self.axis_move = ['z']

                carrot_wp = self.set_carrot_wps(self.trajectory[i],self.currentState[:3])
                self.pid.set_target_pose(carrot_wp,self.axis_move[i])
                self.camera.update_waypoint(carrot_wp)
                
                inputVal = input("Enter 's' to start: ")
                if inputVal=='s':
                    first = False
                    self.comms.arm()
                    self.start_traversal = time.time()
                else:
                    continue
            self.updateAction()
            ret = self.takeAction()

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
  
            if self.pid.isReached():    
                if self.mode == "Rectangle": 
                    i += 1
                    i = min(len(self.trajectory),i)
                    self.pid.update_int_err()
                    
                if i == len(self.trajectory) or self.mode !='Rectangle':
                    print("Reached Final Waypoint.")
                    
                    if self.mode =='Rectangle':
                        print("Now Landing")
                        self.outOfBound = 3
                        self.end_traversal = time.time()
                        print("for traversal", self.end_traversal - self.start_traversal, "s")
                    else:
                        if self.hover_reached_flag:
                            print("Hovering")
                            self.hover_reached_flag = False
                else:
                    self.pid.useWay = True
                # print("IS Reached True")
            if i < len(self.trajectory):
                carrot_wp = self.set_carrot_wps(self.trajectory[i],self.currentState[:3])

                self.pid.set_target_pose(carrot_wp,self.axis_move[i])
                self.camera.update_waypoint(carrot_wp)
            
            if self.debug:
                pass
                # print(f"{time.time()-start_time_pid} s for pid")
            
            loop_time = time.time() - start_time_camera
            if loop_time < self.runLoopWaitTime:
                time.sleep(self.runLoopWaitTime - loop_time)
            
            if self.debug:
                pass
                # print(f"Total time {loop_time}s")
            
        time.sleep(2)
    
    # update currentState
    def updateState(self):
        """
            Parsing the Camera and IMU feed and updating the sensor data to determine the states of the drone for
            pose estimation. Based on a horizon parameter, moving average is applied on the sensor data feed to reduce
            the noise in the data.
        """
        
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
            
            if self.outOfBound==0:
                if self.currentState is  None:
                    self.currentState = list(sensorData[1][:2]) + [sensorData[2]]
                else:
                    if (abs(sensorData[1][0] - self.currentState[0]) > self.state_filter_thresh) or (abs(sensorData[1][1] - self.currentState[1]) > self.state_filter_thresh) or (abs(sensorData[2] - self.currentState[2]) > self.state_filter_thresh):
                        return
                    self.currentState[:3] = list(sensorData[1][:2]) + [sensorData[2]]
                    
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
        
    
    # update action
    def updateAction(self):
        """
        Updating the control actions in terms of the calculated error by the PID and determining the values of
        pitch, roll, throttle and yaw.
        """  
        
        self.pid.update_pos(self.currentState)
        self.pid.calc_err()

        self.action["Pitch"], self.action['Roll'] = self.pid.set_pitch_and_roll()
        self.action["Throttle"] = self.pid.set_thrust()
        self.action["Yaw"] = self.pid.set_yaw()
    
    def takeAction(self):
        """
        Sending the updated control actions using MSP Packets to the drone by setting the control parameters. In case of 
        an undesired state of the drone, landing command is sent to it. For updating the states of the drone, we apply locking
        between the threads, to read the states at one timestep.
        """
        self.commandLock.acquire()
        if self.outOfBound==0:
            # converting to integer as we can only send integral values via MSP Packets
            self.comms.paramsSet["Roll"] = int(self.action["Roll"])
            self.comms.paramsSet["Pitch"] = int(self.action["Pitch"])
            self.comms.paramsSet["Throttle"] = int(self.action["Throttle"])
            self.comms.paramsSet["Yaw"] = int(self.action["Yaw"])
            self.commandLock.release()
            return 0
        else:
            self.comms.paramsSet["currentCommand"] = 2
            self.commandLock.release()
            print("Landing: ",self.outOfBound)
            time.sleep(2)
            self.exit_land()
            return 1
        
    def exit_land(self):
        msg = "Complete + Land"
        self.comms.readLoop = False
        self.comms.writeLoop = False
        self.readThread.join()
        self.writeThread.join()
        cv2.destroyAllWindows()
        print(msg)
        exit()
    
    def handler(self, sigma, frame):
        """
        If the pose estimation for the drone is not recieved for a specified time, it lands the drone and the two threads for        
        writing and reading is joined and code exits.
        """
        msg = "Exit + Land"
        self.comms.land()
        self.comms.readLoop = False
        self.comms.writeLoop = False
        self.readThread.join()
        self.writeThread.join()
        cv2.destroyAllWindows()
        print(msg)
        exit()

if __name__ == '__main__':
    drone1 = autoPluto()
    # drone1.comms.arm()
    drone1.run()
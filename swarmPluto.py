from autoPluto import autoPluto
from configparser import ConfigParser
import time
from vision.vision_pipeline import VisionPipeline
import signal
import cv2

class swarmPluto():
    def __init__(self,debug) :
        signal.signal(signal.SIGINT, self.handler)
        self.debug = debug
        self.config = ConfigParser()
        self.config.read('controls/droneData.ini') 
        self.droneNo1 = "Drone " + str(self.config.getint("Drone Number","drone1"))
        self.droneNo2 = "Drone " + str(self.config.getint("Drone Number","drone2"))
        self.mode =  self.config.get("Mode","mode")
        self.hover_reached_flag = True
        if self.mode == 'Rectangle':
            self.rectangle = [float(i) for i in self.config.get("Rectangle","xyz").split(',')]
        else:
            self.hover_z = float(self.config.get("Hover","z"))
        
        self.markerIdList = [self.config.getint(self.droneNo1,"id"),self.config.getint(self.droneNo2,"id")]
        """
        TODO: 
        1. Add Multi aruco detection based on the list sent
        2. Caliberate Yaw Live
        """
        self.camera = VisionPipeline(rgb_res=(1080,1920),marker_size=3.6,required_marker_id=self.markerIdList,debug=1,padding = 0)
        
        self.trajectory = []
        self.exception = 0
        
        self.drone1 = autoPluto(config=self.config,droneNo=self.droneNo1)
        self.drone2 = autoPluto(config=self.config,droneNo=self.droneNo2)
        
        
        self.done = False
        
        pass
    
    def arm(self):
        self.drone1.arm()
        self.drone2.arm()
    
    def updateStates(self):
        """
        To Do: rewrite after changes in cam_process()
        """
        xyz = self.camera.cam_process()
        if type(xyz) == type(1):
            self.exception = xyz
        else:
            xyz1 = xyz[self.droneNo1]
            xyz2 = xyz[self.droneNo2]
            
            self.drone1.updateState(xyz1)
            self.drone2.updateState(xyz2)
    
    def takeActions(self):
        self.drone1.takeAction(self.exception)
        self.drone2.takeAction(self.exception)
    
    def updateActions(self):
        self.drone1.updateAcion()
        self.drone2.updateAcion()
    
    def updateTargets(self):
        if self.drone1.isReached() and self.drone2.isReached():
            if lastUpdated==2:
                i_target+=1
                if i_target==len(self.trajectory):
                    i_target = 0
                if i_target%2==1:
                    dirOfMotion1 = 'x'
                else:
                    dirOfMotion1 = 'y'
                self.drone1.updateTarget(self.trajectory[i_target],dirOfMotion1)                    
                self.camera.update_waypoint(self.trajectory[i_target],self.markerIdList[0])
                lastUpdated = 1
            elif lastUpdated==1:
                if dirOfMotion1=='x':
                    self.drone2.updateTarget(self.trajectory[i_target-1],'y')
                    self.camera.update_waypoint(self.trajectory[i_target-1],self.markerIdList[1])
                else:
                    self.drone2.updateTarget(self.trajectory[i_target-1],'x')
                    self.camera.update_waypoint(self.trajectory[i_target-1],self.markerIdList[1])
                
                lastUpdated = 2
                if i_target==0:
                    lastUpdated = 0
            elif lastUpdated==0:
                self.done = True
                self.exception = -1
        
    def run(self):
        first = True
        i_target = 0
        dirOfMotion1 = 'z'
        lastUpdated = 2
        while(not self.done):
            """
            Update States
            """
            start_time_camera = time.time()
            self.updateStates()
            
            start_time_pid = time.time()
            if self.debug:
                print(f"{start_time_pid-start_time_camera} s for camera")    
            
            """
            Sanity checks:
            1. State Must be detected
            2. Yaw must be aligned 
            """
            if self.drone1.currentState['x'] is None or self.drone2.currentState['x'] is None:
                continue
            
            """
            Make it live not in init
            """
            if abs(self.camera.cam_rvec[self.markerIdList[0]][2] - self.camera.cam_rvec[self.markerIdList[1]][2]) > 0.01:
                continue
            
            """
            Planning Trajectory
            """
            if first:
                if self.mode == "Rectangle":
                    self.trajectory.append([self.drone1.currentState['x'],  self.drone1.currentState['y'],  self.rectangle[2]])
                    self.trajectory.append([self.drone1.currentState['x']+self.rectangle[0],  self.drone1.currentState['y'],  self.rectangle[2]])
                    self.trajectory.append([self.drone1.currentState['x']+self.rectangle[0],  self.drone1.currentState['y']+self.rectangle[1],  self.rectangle[2]])
                    self.trajectory.append([self.drone1.currentState['x'],  self.drone1.currentState['y']+self.rectangle[1],  self.rectangle[2]])
                else:
                    print("WHYYY HOVERRRR?? (Switch to rectange)")
                
                if abs(self.drone2.currentState['x'] - self.trajectory[-1][0])>0.01 and abs(self.drone2.currentState['y'] - self.trajectory[-1][1])>0.01:
                    continue
                else:
                    self.drone1.updateTarget(self.trajectory[i_target],dirOfMotion1)
                    self.drone2.updateTarget(self.trajectory[i_target-1],dirOfMotion1)
                    self.camera.update_waypoint(self.trajectory[i_target],self.markerIdList[0])
                    self.camera.update_waypoint(self.trajectory[i_target-1],self.markerIdList[1])
                    first = False
            """
            Check Conditions for trajectory waypoint update
            """
            self.updateTargets()
            """
            If both drones have completed trajectory then break and land
            """
            if self.done:
                break
            """
            Update and Take Actions
            """
            self.updateActions()
            self.takeActions()
            
            loop_time = time.time() - start_time_camera
            
            if self.debug:
                print(f"{loop_time-start_time_pid} s for pid")
            
            if self.debug:
                print(f"Total time {loop_time}s")
            
            if loop_time < self.runLoopWaitTime:
                time.sleep(self.runLoopWaitTime - loop_time)
        """
        Make both drones land
        """
        self.takeActions()
        time.sleep(3)
    
    def handler(self, sigma, frame):
        msg = "Exit + Land"
        self.drone1.comms.paramsSet["currentCommand"] = 2
        self.drone2.comms.paramsSet["currentCommand"] = 2
        time.sleep(3)
        self.drone1.closeThreads()
        self.drone2.closeThreads()
        cv2.destroyAllWindows()
        print(msg)
        exit()


if __name__=="__main__":
    swarm = swarmPluto()
    swarm.arm()
    swarm.run()
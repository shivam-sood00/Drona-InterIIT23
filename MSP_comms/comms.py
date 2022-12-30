import socket,time,math,select
from utils import *
class COMMS:
    """ 
    Initialise the socket and define IN & OUT packets   
    """
    def __init__(self,IP='192.168.4.1',Port=23,debug=False):
        self.TCP_IP = IP
        self.Port = Port
        self.debug=debug
        self.mySocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.mySocket.connect((self.TCP_IP, self.Port))
        if(self.debug):
            print("Socket Connected")
        self.IN = IN_PACKETS(self.mySocket,debug=self.debug)
        self.OUT = OUT_PACKETS(self.mySocket,debug=self.debug)
        self.params = {}
    
    def disconnect(self):
        self.mySocket.close()
    
    """ 
    Target Function for the readThread
    """
    def updateParams(self):
        buff = []

        # Always reading values and updating the buffer within the readThread
        while(True):
            ready = select.select([self.mySocket],[],[],2) # Time out after 2 seconds of not getting data
            if not ready[0]:
                break
            out,idx,buff = self.OUT.receiveMSPresponse(buff) # Recieving the MSP Packets
            if self.debug:
                print("out,idx,buff: ",out,idx,buff)
            self.updateParamsIdx(out,idx) # Updating params based on their indices for OUT Packets
    
    def printParams(self):
        print("Params: ",self.params)
    
    def updateParams(self,out,idx):
        if idx==108: #MSP_ATTITUDE
            self.params["Roll"]=out[0]
            self.params["Pitch"]=out[1]
            self.params["Yaw"]=out[2]
        elif idx==109: #MSP_ALTITUDE
            self.params["Altitude"]=out[0]
            self.params["Vario"]=out[1]
        elif idx==102: #MSP_RAW_IMU
            self.params["AccX"]=out[0]
            self.params["AccY"]=out[1]
            self.params["AccZ"]=out[2]
            self.params["GyroX"]=out[3]
            self.params["GyroY"]=out[4]
            self.params["GyroZ"]=out[5]
            self.params["MagX"]=out[6]
            self.params["MagY"]=out[7]
            self.params["MagZ"]=out[8]
        
        if self.debug:
            self.printParams()
    

class IN_PACKETS:
    def __init__(self,mySocket,debug=False):
        self.mySocket = mySocket
        self.debug = debug
        #header(2 bytes), 60 - to the controller, 62 - from the controller
        self.headerArray=bytearray([36,77,60])
        self.armed = False

    def default_RC(self):
        valueArray = bytearray([])
        valueArray.extend(self.headerArray)
        msgLen = 16
        valueArray.append(msgLen) #MSG LENGTH
        valueArray.append(200) #MSG TYPE FOR RAW_RC
        roll=1500                    
        pitch=1500                 
        throttle=1800 
        yaw=1500                      
        aux1=1000
        aux2=1500
        aux3=1500
        if self.armed==True:
            value=1500
        else:
            value=1000
        aux4=value
        # print("AUX4: ",aux4)
        valueArray.extend(getBytes(roll))
        valueArray.extend(getBytes(pitch))
        valueArray.extend(getBytes(throttle))
        valueArray.extend(getBytes(yaw))
        valueArray.extend(getBytes(aux1))
        valueArray.extend(getBytes(aux2))
        valueArray.extend(getBytes(aux3))
        valueArray.extend(getBytes(aux4))
        return valueArray
    
    def default_CMD(self):
        valueArray = bytearray([])
        valueArray.extend(self.headerArray)
        valueArray.append(2) #MSG LENGTH
        valueArray.append(217) #MSG TYPE FOR MSP_SET_COMMAND
        return valueArray
    
    def setRoll(self,value):
        valueArray = self.default_RC()
        bytes = getBytes(value)
        valueArray[5] = bytes[0]
        valueArray[6] = bytes[1]
        valueArray.append(getCRC(valueArray))
        if(self.debug):
            print("Roll set \nSending Packet: ",list(valueArray))
        self.mySocket.send(valueArray)

    def setPitch(self,value):
        valueArray = self.default_RC()
        bytes = getBytes(value)
        valueArray[7] = bytes[0]
        valueArray[8] = bytes[1]
        valueArray.append(getCRC(valueArray))
        if(self.debug):
            print("Pitch set \nSending Packet: ",list(valueArray))
        self.mySocket.send(valueArray)
    
    def setThrottle(self,value):
        valueArray = self.default_RC()
        bytes = getBytes(value)
        valueArray[9] = bytes[0]
        valueArray[10] = bytes[1]
        valueArray.append(getCRC(valueArray))
        if(self.debug):
            print("Throttle set \nSending Packet: ",list(valueArray))
        self.mySocket.send(valueArray)
    
    def setYaw(self,value):
        valueArray = self.default_RC()
        bytes = getBytes(value)
        valueArray[11] = bytes[0]
        valueArray[12] = bytes[1]
        valueArray.append(getCRC(valueArray))
        if(self.debug):
            print("Yaw set \nSending Packet: ",list(valueArray))
        self.mySocket.send(valueArray)
    
    def setMagMode(self,value):
        if value==True:
            value=1000
        else:
            value=1500
        valueArray = self.default_RC()
        bytes = getBytes(value)
        valueArray[13] = bytes[0]
        valueArray[14] = bytes[1]
        valueArray.append(getCRC(valueArray))
        if(self.debug):
            if(value==1000):
                print("MagMode set \nSending Packet: ",list(valueArray))
            else:
                print("HeadFreeMode set \nSending Packet: ",list(valueArray))
        self.mySocket.send(valueArray)
    
    def setDevMode(self,value):
        if value==True:
            value=1500
        else:
            value=1000
        valueArray = self.default_RC()
        bytes = getBytes(value)
        valueArray[15] = bytes[0]
        valueArray[16] = bytes[1]
        valueArray.append(getCRC(valueArray))
        if(self.debug):
            if(value==1500):
                print("DevMode set \nSending Packet: ",list(valueArray))
            else:
                print("DevMode removed \nSending Packet: ",list(valueArray))
        self.mySocket.send(valueArray)
    
    def setAltHold(self,value):
        if value==True:
            value=1500
        else:
            value=1000
        valueArray = self.default_RC()
        bytes = getBytes(value)
        valueArray[17] = bytes[0]
        valueArray[18] = bytes[1]
        valueArray.append(getCRC(valueArray))
        if(self.debug):
            if(value==1500):
                print("Alt Mode set \nSending Packet: ",list(valueArray))
            else:
                print("Throttle Range (not Alt Mode) \nSending Packet: ",list(valueArray))
        self.mySocket.send(valueArray)
    
    def Arm(self,value):
        self.armed=value
        if self.armed==True:
            value=1500
        else:
            value=1000
        valueArray = self.default_RC()
        bytes = getBytes(value)
        valueArray[19] = bytes[0]
        valueArray[20] = bytes[1]
        valueArray.append(getCRC(valueArray))
        if(self.debug):
            if(value==1500):
                print("Drone Armed \nSending Packet: ",list(valueArray))
            else:
                print("Drone Disarmed \nSending Packet: ",list(valueArray))
        self.mySocket.send(valueArray)
        
    def takeOff(self):
        valueArray = self.default_CMD()
        valueArray.extend(getBytes(1))
        valueArray.append(getCRC(valueArray))
        if self.debug:
            print("Take-Off\nsent packet: ",list(valueArray))
        self.mySocket.send(valueArray)
    
    def land(self):
        valueArray = self.default_CMD()
        valueArray.extend(getBytes(2))
        valueArray.append(getCRC(valueArray))
        if self.debug:
            print("Land")
        self.mySocket.send(valueArray)
    
    def backFlip(self):
        valueArray = self.default_CMD()
        valueArray.extend(getBytes(3))
        valueArray.append(getCRC(valueArray))
        if self.debug:
            print("Back-Flip")
        self.mySocket.send(valueArray)

    def frontFlip(self):
        valueArray = self.default_CMD()
        valueArray.extend(getBytes(4))
        valueArray.append(getCRC(valueArray))
        if self.debug:
            print("Front-Flip")
        self.mySocket.send(valueArray)
    
    def rightFlip(self):
        valueArray = self.default_CMD()
        valueArray.extend(getBytes(5))
        valueArray.append(getCRC(valueArray))
        if self.debug:
            print("Right-Flip")
        self.mySocket.send(valueArray)
        
    def leftFlip(self):
        valueArray = self.default_CMD()
        valueArray.extend(getBytes(6))
        valueArray.append(getCRC(valueArray))
        if self.debug:
            print("Left-Flip")
        self.mySocket.send(valueArray)

class OUT_PACKETS:
    def __init__(self,mySocket,bufferSize=64,debug=False):
        self.mySocket = mySocket
        self.debug = debug
        self.bufferSize = bufferSize
        self.headerArray=bytearray([36,77,60])
        self.headerArrayOut=bytearray([36,77,62])
        # print(self.debug," in out Packets")
        
    def requestMSPAttitude(self):
        valueArray = bytearray([])
        valueArray.extend(self.headerArray)
        valueArray.append(0)
        valueArray.append(108)
        valueArray.append(getCRC(valueArray))
        if self.debug:
            print("Request for MSP Attitude \nSent Packet: ",list(valueArray))
        self.mySocket.send(valueArray)
        
    def requestMSPAltitude(self):
        valueArray = bytearray([])
        valueArray.extend(self.headerArray)
        valueArray.append(0)
        valueArray.append(109)
        valueArray.append(getCRC(valueArray))
        if self.debug:
            print("Request for MSP Altitude \nSent Packet: ",list(valueArray))
        self.mySocket.send(valueArray)
    
    def requestMSPRawIMU(self):
        valueArray = bytearray([])
        valueArray.extend(self.headerArray)
        valueArray.append(0)
        valueArray.append(102)
        valueArray.append(getCRC(valueArray))
        if self.debug:
            print("Request for MSP Altitude \nSent Packet: ",list(valueArray))
        self.mySocket.send(valueArray)
    
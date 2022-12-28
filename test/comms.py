import socket,time,math,select
from utlis import *
class COMMS:
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
    
    def disconnect(self):
        self.mySocket.close()
class IN_PACKETS:
    def __init__(self,mySocket,debug=False):
        self.mySocket = mySocket
        self.debug = debug
        #header(2 bytes), 60 - to the controller, 62 - from the controller
        self.headerArray=bytearray([36,77,60])

    def default_RC(self):
        valueArray = bytearray([])
        valueArray.extend(self.headerArray)
        msgLen = 16
        valueArray.append(msgLen) #MSG LENGTH
        valueArray.append(200) #MSG TYPE FOR RAW_RC
        roll=1500                    
        pitch=1500                 
        throttle=1500 
        yaw=1500                      
        aux1=1000
        aux2=1500
        aux3=1500
        aux4=1500
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
        valueArray.extend(getBytes(2)) #MSG LENGTH
        valueArray.extend(getBytes(217)) #MSG TYPE FOR MSP_SET_COMMAND
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
        valueArray.append(getCRC(valueArray,self.debug))
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
        if value==True:
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
            print("Take-Off")
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
        print(self.debug," in out Packets")
        
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
    
    def requestMSPAltitude(self):
        valueArray = bytearray([])
        valueArray.extend(self.headerArray)
        valueArray.append(0)
        valueArray.append(102)
        valueArray.append(getCRC(valueArray))
        if self.debug:
            print("Request for MSP Altitude \nSent Packet: ",list(valueArray))
        self.mySocket.send(valueArray)
    
    def receiveMSPAttitude(self,arr):
        arr = []
        while(True):
            ready = select.select([self.mySocket],[],[],2) # Time out after 2 seconds of not getting data
            if not ready[0]:
                orientation = -1
                break
            arr1 = self.mySocket.recv(self.bufferSize)
            arr+=list(arr1)[:]
            if(self.debug):
                print(arr)
            
            if len(arr)==12:
                l = []
                for i in range(5,11,2):
                    l.append(toDec(arr[i],arr[i+1],(i-5)/2))
                l[0] = l[0]/10
                l[1] = l[1]/10
                if self.debug:
                    print("\n")
                    print("roll: ",l[0]," degrees")
                    print("pitch: ",l[1]," degrees")
                    print("yaw: ",l[2]," degrees")
                orientation =  l
            else:
                orientation = 0
            # orientation = comms.OUT.receiveMSPAttitude(arr)
            if(orientation!=-1):
                if(type(orientation)!=type(1)):
                    arr = []
                print("orientation: ",orientation)
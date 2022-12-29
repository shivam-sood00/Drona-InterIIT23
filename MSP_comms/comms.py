import socket,time,math,select
from utils import *

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
        self.params = [0]*14
    
    def disconnect(self):
        self.mySocket.close()
    
    def updateParams(self):
        buff = []
        while(True):
            ready = select.select([self.mySocket],[],[],2) # Time out after 2 seconds of not getting data
            if not ready[0]:
                break
            out,idx,buff = self.OUT.receiveMSPresponse(buff)
            if self.debug:
                print("out,idx,buff: ",out,idx,buff)
            self.updateParamsIdx(out,idx)
    
    def printParams(self):
        print("Params: ",self.params)
    
    def updateParamsIdx(self,out,idx):
        if idx==108:
            self.params[0]=out[0]
            self.params[1]=out[1]
            self.params[2]=out[2]
        elif idx==109:
            self.params[3]=out[0]
            self.params[4]=out[1]
        elif idx==102:
            self.params[5]=out[0]
            self.params[6]=out[1]
            self.params[7]=out[2]
            self.params[8]=out[3]
            self.params[9]=out[4]
            self.params[10]=out[5]
            self.params[11]=out[6]
            self.params[12]=out[7]
            self.params[13]=out[8]
        
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
        print(aux4)
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
    
    def receiveMSPresponse(self,buff):            
        arr = self.mySocket.recv(self.bufferSize)
        buff += list(arr)
        if(self.debug):
            print("buff: ",buff)
        
        if len(buff)<5:
            return [],0,buff
        
        for i in range(3):
            if buff[i]!=self.headerArrayOut[i]:
                if i==2:
                    if buff[i]==33:
                        print("Error sent in out packet....!!!!!")
                        return
                    else:
                        buff = buff[2:]
                else:
                    print("garbage received (even header does not match)..!!")
                    return
        
        msgLen = buff[3]
        if msgLen==0:
            buff= buff[6:]
            if self.debug:
                print("Ignoring 0 len message..!!")
            return [],0,buff
        
        elif len(buff)>=msgLen+6:
            idx = buff[4]
            checksum = msgLen^idx
            
            out = []
            if idx==108:
                for i in range(0,msgLen,2):
                    out.append(getSignedDec(buff[i+5],buff[i+6],i/2))
            elif idx==109:
                lsb16 = getDec(buff[5],buff[6])
                msb16 = getDec(buff[7],buff[8])
                # dividing by 100 to convert to meters and meter-per-second
                out.append(getDec(lsb16,msb16,256*256)/100)
                out.append(getSignedDec(buff[9],buff[10],0)/100)
            elif idx==102:
                for i in range(0,msgLen,2):
                    out.append(getSignedDec(buff[i+5],buff[i+6]))
            
            for i in range(msgLen):
                checksum ^= buff[i+5]
            
            if checksum==buff[msgLen+5]:
                buff = buff[msgLen+6:]
                if self.debug:
                    print("successfully decoded!!\nout: ",out,"\n idx: ",idx,"msgLen: ",msgLen)
                return out,idx,buff
            else:
                print("Error in decoding the buffer....!!!!")
                return
        else:
            return [],0,buff
import socket,select,time
from utils import *


class MSPPacket:
    def __init__(self):
        self.header = [36, 77]
        self.direction = {"in":60,"out":62} 
        self.msg = []
    
    def getInMsgRequest(self,msgLen,typeOfPayload,msgData):
        self.msg += self.header
        self.msg.append(self.direction["in"])
        self.msg.append(msgLen)
        self.msg.append(typeOfPayload)
        # print(msgData)
        self.msg.extend(msgData)
        # print(self.msg)
        checksum = 0
        for i in range(3,len(self.msg),1):
            checksum ^= self.msg[i]
        self.msg.append(checksum)
        return self.msg
    
    def getOutMsgRequest(self,msgLen,typeOfPayload,msgData):
        self.msg += self.header
        self.msg.append(self.direction["out"])
        self.msg.append(msgLen)
        self.msg.append(typeOfPayload)
        self.msg.extend(msgData)
        # print(self.msg)
        checksum = 0
        for i in range(3,len(self.msg),1):
            checksum ^= self.msg[i]
        self.msg.append(checksum)
        return self.msg

class COMMS:
    def __init__(self,IP='192.168.4.1',Port=23,debug=False):
        self.TCP_IP = IP
        self.Port = Port
        self.debug=debug
        self.mySocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.mySocket.connect((self.TCP_IP, self.Port))
        self.inLoopSleepTime = 0.022
        self.outLoopSleepTime = 0
        self.sendData = False
        self.requests = []
        self.outServices = [108,109,102]
        self.outBufferSize = 64
        if(self.debug):
            print("Socket Connected")
        self.paramsSet = {"Roll":1500,"Pitch":1500,"Throttle":1500,"Yaw":1500,"Aux1":1500,"Aux2":1500,"Aux3":1500,"Aux4":1000,"currentCommand":0}
        self.paramsReceived = {"Roll":-1,"Pitch":-1,"Yaw":-1,"Altitude":-1,"Vario":-1,"AccX":-1,"AccY":-1,"AccZ":-1,"GyroX":-1,"GyroY":-1,"GyroZ":-1,"MagX":-1,"MagY":-1,"MagZ":-1}
    
    def arm(self):
        self.paramsSet["Roll"] = 1500
        self.paramsSet["Pitch"] = 1500
        self.paramsSet["Throttle"] = 1000
        self.paramsSet["Aux4"] = 1500
        time.sleep(1)
    
    def boxArm(self):
        self.paramsSet["Roll"] = 1500
        self.paramsSet["Pitch"] = 1500
        self.paramsSet["Throttle"] = 1500
        self.paramsSet["Aux4"] = 1500
        time.sleep(1)
    
    def disArm(self):
        self.paramsSet["Throttle"] = 1300
        self.paramsSet["Aux4"] = 1000
        time.sleep(1)
    
    def forward(self):
        self.paramsSet["Pitch"] = 1600
        time.sleep(1)
        
    def backward(self):
        self.paramsSet["Pitch"] = 1400
        time.sleep(1)
        
    def left(self):
        self.paramsSet["Roll"] = 1400
        time.sleep(1)
        
    def right(self):
        self.paramsSet["Roll"] = 1600
        time.sleep(1)
        
    def leftYaw(self):
        self.paramsSet["Yaw"] = 1200
        time.sleep(1)
    
    def rightYaw(self):
        self.paramsSet["Yaw"] = 1800
        time.sleep(1)
    
    def reset(self):
        self.paramsSet["Roll"] = 1500
        self.paramsSet["Pitch"] = 1500
        self.paramsSet["Throttle"] = 1500
        self.paramsSet["Yaw"] = 1500
        self.paramsSet["currentCommand"] = 0
        time.sleep(1)
    
    def increaseHeight(self):
        self.paramsSet["Throttle"] = 1800
        time.sleep(1)
    
    def decreaseHeight(self):
        self.paramsSet["Throttle"] = 1300
        time.sleep(1)
    
    def takeOff(self):
        self.disArm()
        self.boxArm()
        self.paramsSet["currentCommand"] = 1
        time.sleep(1)
    
    def land(self):
        self.paramsSet["currentCommand"] = 2
        time.sleep(2)
    
    def getAllRequestMsgs(self):
        for service in self.outServices:
            self.requests.append(MSPPacket().getInMsgRequest(0,service,[]))

    def write(self):
        self.sendData = True
        self.getAllRequestMsgs()
        while(self.sendData):
            print("Throttle",self.paramsSet["Throttle"])
            msgLen = 16
            typeOfPayload = 200
            msgData = []
            msgData.extend(getBytes(self.paramsSet["Roll"]))
            msgData.extend(getBytes(self.paramsSet["Pitch"]))
            msgData.extend(getBytes(self.paramsSet["Throttle"]))
            msgData.extend(getBytes(self.paramsSet["Yaw"]))
            msgData.extend(getBytes(self.paramsSet["Aux1"]))
            msgData.extend(getBytes(self.paramsSet["Aux2"]))
            msgData.extend(getBytes(self.paramsSet["Aux3"]))
            msgData.extend(getBytes(self.paramsSet["Aux4"]))
            msgToBeSent = MSPPacket().getInMsgRequest(msgLen,typeOfPayload,msgData)
            print(msgToBeSent)
            self.mySocket.send(bytearray(msgToBeSent))
            if self.paramsSet["currentCommand"]!=0:
                msgLen = 2
                typeOfPayload = 217
                msgData = []
                msgData.extend(getBytes(self.paramsSet["currentCommand"]))
                self.paramsSet["currentCommand"]=0
                msgToBeSent = MSPPacket().getInMsgRequest(msgLen,typeOfPayload,msgData);
                print(msgToBeSent)
                self.mySocket.send(bytearray(msgToBeSent))
            for request in self.requests:
                self.mySocket.send(bytearray(request))
            time.sleep(self.inLoopSleepTime)

    
    def read(self):
        buff = []
        # Always reading values and updating the buffer within the readThread
        while(True):
            ready = select.select([self.mySocket],[],[],2) # Time out after 2 seconds of not getting data
            if not ready[0]:
                break
            out,idx,buff = self.receiveMSPresponse(buff) # Recieving the MSP Packets
            if self.debug:
                print("out,idx,buff: ",out,idx,buff)
            self.updateParams(out,idx) # Updating params based on their indices for OUT Packets
            time.sleep(self.outLoopSleepTime)
    
    def printParams(self):
        print("Params: ",self.params)
    
    def updateParams(self,out,idx):
        if idx==108: #MSP_ATTITUDE
            self.paramsReceived["Roll"]=out[0]
            self.paramsReceived["Pitch"]=out[1]
            self.paramsReceived["Yaw"]=out[2]
        elif idx==109: #MSP_ALTITUDE
            self.paramsReceived["Altitude"]=out[0]
            self.paramsReceived["Vario"]=out[1]
        elif idx==102: #MSP_RAW_IMU
            self.paramsReceived["AccX"]=out[0]
            self.paramsReceived["AccY"]=out[1]
            self.paramsReceived["AccZ"]=out[2]
            self.paramsReceived["GyroX"]=out[3]
            self.paramsReceived["GyroY"]=out[4]
            self.paramsReceived["GyroZ"]=out[5]
            self.paramsReceived["MagX"]=out[6]
            self.paramsReceived["MagY"]=out[7]
            self.paramsReceived["MagZ"]=out[8]
        
        if self.debug:
            self.printParams()
    
    def receiveMSPresponse(self,buff):            
        arr = self.mySocket.recv(self.outBufferSize)
        buff += list(arr)
        if(self.debug):
            print("buff: ",buff)
        
        if len(buff)<5:
            return [],0,buff
        
        headerArrayOut = [36,77,62]
        for i in range(3):
            if buff[i]!=headerArrayOut[i]:
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

    def disconnect(self):
        self.mySocket.close()
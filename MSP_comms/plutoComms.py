import socket,select,time
from utils import *


"""
Module for MSP Packet
"""
class MSPPacket:
    def __init__(self,debug):
        self.debug = debug
        self.header = [36, 77]
        self.direction = {"in":60,"out":62} 
        self.msg = []
    
    # Appends CRC at the end of message. This is to check the integrity of the packet
    def appendCRC(self):
        checksum = 0
        for i in range(3,len(self.msg),1):
            checksum ^= self.msg[i]
        self.msg.append(checksum)
    
    # Returns MSP Packet of in direction for given message length, type of payload and data
    def getInMsgRequest(self,msgLen,typeOfPayload,msgData):
        self.msg += self.header
        self.msg.append(self.direction["in"])
        self.msg.append(msgLen)
        self.msg.append(typeOfPayload)
        self.msg.extend(msgData)
        self.appendCRC()
        if self.debug:
            print(self.msg)
        return self.msg
    
    # Returns MSP Packet of out direction for given message length, type of payload and data
    def getOutMsgRequest(self,msgLen,typeOfPayload,msgData):
        self.msg += self.header
        self.msg.append(self.direction["out"])
        self.msg.append(msgLen)
        self.msg.append(typeOfPayload)
        self.msg.extend(msgData)
        self.appendCRC()
        if self.debug:
            print(self.msg)
        return self.msg

"""
Module to facilitate all communication with the pluto drone
"""
class COMMS:
    def __init__(self,IP='192.168.4.1',Port=23,debug=False):
        self.TCP_IP = IP
        self.Port = Port
        self.debug=debug
        self.mySocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.mySocket.connect((self.TCP_IP, self.Port))
        self.waitTime = 0.1
        if(self.debug):
            print("Socket Connected")
        # Variable to control writing and reading frequency
        self.inLoopSleepTime = 0.022
        self.outLoopSleepTime = 0
        # Variable to control writing loop i.e. if we want to write data or not
        self.writeLoop = False
        self.readLoop = False
        # list of all type of Payload of all the OUT Packets to be requested
        self.outServices = {"MSP_ATTITUDE":108,"MSP_ALTITUDE":109,"MSP_RAW_IMU":102}
        # list to store MSP messages of the OUT Packets we request
        self.requestMSPPackets = []
        # variable to control buffer size of OUT loop 
        self.outBufferSize = 64
        # Dictionary of all the parameters we write to the pluto drone
        self.paramsSet = {"Roll":1500,"Pitch":1500,"Throttle":1500,"Yaw":1500,"Aux1":1500,"Aux2":1500,"Aux3":1500,"Aux4":1000,"currentCommand":0}
        # Dictionary of all the parameters we read from the pluto drone
        self.paramsReceived = {"Roll":-1,"Pitch":-1,"Yaw":-1,"Altitude":-1,"Vario":-1,"AccX":-1,"AccY":-1,"AccZ":-1,"GyroX":-1,"GyroY":-1,"GyroZ":-1,"MagX":-1,"MagY":-1,"MagZ":-1}
    
    def arm(self):
        self.paramsSet["Roll"] = 1500
        self.paramsSet["Pitch"] = 1500
        self.paramsSet["Throttle"] = 1000
        self.paramsSet["Aux4"] = 1500
        time.sleep(self.waitTime)
    
    def boxArm(self):
        self.paramsSet["Roll"] = 1500
        self.paramsSet["Pitch"] = 1500
        self.paramsSet["Throttle"] = 1500
        self.paramsSet["Aux4"] = 1500
        time.sleep(self.waitTime)
    
    def disArm(self):
        self.paramsSet["Throttle"] = 1300
        self.paramsSet["Aux4"] = 1000
        time.sleep(self.waitTime)
    
    def forward(self):
        self.paramsSet["Pitch"] = 1600
        time.sleep(self.waitTime)
        
    def backward(self):
        self.paramsSet["Pitch"] = 1400
        time.sleep(self.waitTime)
        
    def left(self):
        self.paramsSet["Roll"] = 1400
        time.sleep(self.waitTime)
        
    def right(self):
        self.paramsSet["Roll"] = 1600
        time.sleep(self.waitTime)
        
    def leftYaw(self):
        self.paramsSet["Yaw"] = 1200
        time.sleep(self.waitTime)
    
    def rightYaw(self):
        self.paramsSet["Yaw"] = 1800
        time.sleep(self.waitTime)
    
    def reset(self):
        self.paramsSet["Roll"] = 1500
        self.paramsSet["Pitch"] = 1500
        self.paramsSet["Throttle"] = 1500
        self.paramsSet["Yaw"] = 1500
        self.paramsSet["currentCommand"] = 0
        time.sleep(self.waitTime)
    
    def increaseHeight(self):
        self.paramsSet["Throttle"] = 1800
        time.sleep(self.waitTime)
    
    def decreaseHeight(self):
        self.paramsSet["Throttle"] = 1300
        time.sleep(self.waitTime)
    
    def takeOff(self):
        self.disArm()
        self.boxArm()
        self.paramsSet["currentCommand"] = 1
        time.sleep(self.waitTime)
    
    def land(self):
        self.reset()
        self.paramsSet["currentCommand"] = 2
        # Giving extra 2 seconds to land
        time.sleep(self.waitTime+2)
        self.disArm()
    
    def backFlip(self):
        self.paramsSet["currentCommand"] = 3
        time.sleep(self.waitTime)
    
    # function to generate MSP Packets for all OUT services 
    def getAllRequestMsgs(self):
        for service in self.outServices.values():
            self.requestMSPPackets.append(MSPPacket().getInMsgRequest(0,service,[]))

    """
    Target of the Writing Thread
    ALl writing to drone takes place in this function untill we set sendData to false
    """
    def write(self):
        self.writeLoop = True
        self.getAllRequestMsgs()
        while(self.writeLoop):
            # Sending MSP_SET_RAW_RC type message
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
            if self.debug:
                print(msgToBeSent)
            self.mySocket.send(bytearray(msgToBeSent))
            # Sending MSP_SET_COMMAND type message
            if self.paramsSet["currentCommand"]!=0:
                msgLen = 2
                typeOfPayload = 217
                msgData = []
                msgData.extend(getBytes(self.paramsSet["currentCommand"]))
                self.paramsSet["currentCommand"]=0
                msgToBeSent = MSPPacket().getInMsgRequest(msgLen,typeOfPayload,msgData);
                if self.debug:
                    print(msgToBeSent)
                self.mySocket.send(bytearray(msgToBeSent))
            # sending MSP request messages for OUT Packets
            for request in self.requestMSPPackets:
                self.mySocket.send(bytearray(request))
            # sleep to control writing frequency
            time.sleep(self.inLoopSleepTime)

    """
    Target function to the reading thread
    All the data sent by the drone is received in this function
    """
    def read(self):
        # buffer to hold the data sent by drone
        buff = []
        self.readLoop = True
        # Always reading values and updating the buffer within the readThread utill readLoop is set to false
        while(self.readLoop):
            """
            
            commented as of now, if code works without this then this should be removed 
            
            # Time out after 2 seconds of not getting data
            ready = select.select([self.mySocket],[],[],2) 
            if not ready[0]:
                break
            """
            # receives the response and processes the buffer
            out,idx,buff = self.receiveMSPresponse(buff) 
            if self.debug:
                print("out,idx,buff: ",out,idx,buff)
            # Updating params based on the OUT Packet received from drone
            self.updateParams(out,idx) 
            # variable to handle the reading frequency
            time.sleep(self.outLoopSleepTime)
    
    # function to print all the parameters
    def printParams(self):
        print("ParamsSet: ",self.paramsSet)
        print("ParamsReceived: ",self.paramsReceived)
    
    # function to update the received parameters
    def updateParams(self,out,idx):
        if idx==self.paramsReceived["MSP_ATTITUDE"]: #MSP_ATTITUDE
            self.paramsReceived["Roll"]=out[0]
            self.paramsReceived["Pitch"]=out[1]
            self.paramsReceived["Yaw"]=out[2]
        elif idx==self.paramsReceived["MSP_ALTITUDE"]: #MSP_ALTITUDE
            self.paramsReceived["Altitude"]=out[0]
            self.paramsReceived["Vario"]=out[1]
        elif idx==elf.paramsReceived["MSP_RAW_IMU"]: #MSP_RAW_IMU
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
    
    # Reads data and appends it at the end of buffer 
    def updateBuffer(self,buff):
        arr = self.mySocket.recv(self.outBufferSize)
        buff += list(arr)
        if(self.debug):
            print("buff: ",buff)
        return buff
    
    """
    This function processes the current buffer and return the list of decoded values as out list, 
    the OUT Packet payload corresponding to which the values are and the remaining buffer.
    Returns empty out list and 0 as payload if buffer does not contain the full message
    """
    def processBuffer(self,buff):
        
        # If length of buffer is less than 5 then it cannot conatin a message, so return buffer as it is
        if len(buff)<5:
            return [],0,buff
        
        # Checking the header and direction of message, if it does not match we report an error or handle accordingly
        headerArrayOut = [36,77,62]
        for i in range(3):
            if buff[i]!=headerArrayOut[i]:
                if i==2:
                    if buff[i]==33:
                        print("Error sent in out packet....!!!!!")
                        return
                    else:
                        buff = buff[2:]
                        return [],0,buff
                else:
                    print("garbage received (even header does not match)..!!")
                    return
        
        msgLen = buff[3]
        # handling the case of empty message
        if msgLen==0:
            buff= buff[6:]
            if self.debug:
                print("Ignoring 0 len message..!!")
            return [],0,buff
        
        # case when complete message if present
        elif len(buff)>=msgLen+6:
            idx = buff[4]
            
            out = []
            if idx==self.outServices["MSP_ATTITUDE"]:
                for i in range(0,msgLen,2):
                    out.append(getSignedDec(buff[i+5],buff[i+6],i/2))
            elif idx==self.outServices["MSP_ALTITUDE"]:
                lsb16 = getDec(buff[5],buff[6])
                msb16 = getDec(buff[7],buff[8])
                # dividing by 100 to convert to meters and meter-per-second
                out.append(getDec(lsb16,msb16,256*256)/100)
                out.append(getSignedDec(buff[9],buff[10],0)/100)
            elif idx==self.outServices["MSP_RAW_IMU"]:
                for i in range(0,msgLen,2):
                    out.append(getSignedDec(buff[i+5],buff[i+6]))
            
            checksum = msgLen^idx
            for i in range(msgLen):
                checksum ^= buff[i+5]
            
            # checking integrity of packet, if damaged we return that there is an error
            if checksum==buff[msgLen+5]:
                buff = buff[msgLen+6:]
                if self.debug:
                    print("successfully decoded!!\nout: ",out,"\n idx: ",idx,"msgLen: ",msgLen)
                return out,idx,buff
            else:
                print("Error in decoding the buffer....!!!!")
                return
        else:
            # return buffer as it is if complete message is not present
            return [],0,buff
    
    # function to receive data and process it to return decoded values and updated buffer
    def receiveMSPresponse(self,buff):            
        buff = self.updateBuffer(buff)
        return self.processBuffer(buff)

    # function to disconnect with the communication
    def disconnect(self):
        self.readLoop = False
        self.writeLoop = False
        self.mySocket.close()
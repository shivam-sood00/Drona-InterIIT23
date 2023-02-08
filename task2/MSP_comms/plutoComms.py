import socket,select,time
from task2.MSP_comms.utils import *

"""
Module for MSP Packet
"""
class MSPPacket:
    """
    This is a class for MSP Packets.
    
    Attributes:
        header: a list containing the header of the MSP Packet i.e. [36, 77].
        direction: a dictionart to store the value mapped to in and out direction of MSGS.
        msg: a list to store the full message.
        debug: a flag to enable prints in code for debugging.
    """
    def __init__(self,debug=False):
        """
        Constructor for the MSP Packet Class.
        Initializes the header list and direction dictionary.
        
        Parameters:
            debug: to pass in the flag for debugging, by default set to False.
        """
        self.debug = debug
        self.header = [36, 77]
        self.direction = {"in":60,"out":62} 
        self.msg = []
    
    # Appends CRC at the end of message. This is to check the integrity of the packet
    def appendCRC(self):
        """
        Constructor for the MSP Packet Class.
        Initializes the header list and direction dictionary.
        
        Parameters:
            debug: to pass in the flag for debugging, by default set to False.
        """
        checksum = 0
        for i in range(3,len(self.msg),1):
            # print(type(self.msg[i]))
            checksum ^= self.msg[i]
        self.msg.append(checksum)
    
    # Returns MSP Packet of in direction for given message length, type of payload and data
    def getInMsgRequest(self,msgLen,typeOfPayload,msgData):
        """
        This is a Member Function to return MSP Packet in direction for given message length, type of payload and data.
        
        Parameter:
            msgLen: length of message.
            typeOfPayload: stores value assigned to that type of payload.
            msgData: stores the data to be sent.
        """
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
        """
        This is a Member Function to return MSP Packet out direction for given message length, type of payload and data.
        
        Parameter:
            msgLen: length of message.
            typeOfPayload: stores value assigned to that type of payload.
            msgData: stores the data to be sent.
        """
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
Module to facilitate all communication with the pluto drone    time.sleep(5)
    # drone.paramsSet["trimRoll"] = 5
    # drone.paramsSet["trimPitch"] = 5
    # time.sleep(5)
    # drone.paramsSet["trimRoll"] = 1
    # drone.paramsSet["trimPitch"] = 1
"""
class COMMS:
    """
    This is the class for all communication with the pluto drone.
    
    It has two threads running in parallel:
    1) The write Thread: which sends data to the drone based on the values of the parameter to be set and requests to be made.
    2) The read Thread: which reads data from the drone and updates the parameters received based on the requests sent.
    
    Attribute:
        TCP_IP: IP address of the drone.
        Port: Port of the drone.
        debug: a flag to enable prints in code for debugging.
        mySocket: refers to the instance of socket connected to the drone at the specified IP and Port.
        waitTime: variable to control sleep time after every command, so as to ensure that it is sent to the drone in the write thread.
        inLoopSleepTime: variable to control sleep time in write thread, thus controlling publishing frequency.
        outLoopSleepTime: variable to control sleep time in read thread, thus controlling subscribing frequency.
        writeLoop: variable to control writing loop i.e. if we want to write data or not.
        readLoop: variable to control reading loop i.e. if we want to read data or not.
        outServices: Dictionary of all the out packets we request for with their type of payload values.
        requestMSPPackets: List of MSP requests for Out Packets. 
        paramSet: Dictionary of parameter to be set based on their values through IN Packets.
        paramReceived: Dictionary of parameters received from the OUT Packets.    
    
    """
    def __init__(self,IP='192.168.4.1',Port=23,debug=False):
        """
        Constructor for the COMMS class.
        
        Parameters:
            :param IP: IP Addresss to connect to, defaults to '192.168.4.1'.
            :type IP: str, optional.
            :param Port: Port to connect to, defaults to 23.
            :type Port: int, optional.
            :param debug: to pass in the flag for debugging, defaults to False.
            :type debug: bool, optional.
        """
        self.TCP_IP = IP
        self.Port = Port
        self.debug=debug
        self.mySocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.mySocket.connect((self.TCP_IP, self.Port))
        self.waitTime = 0.17
        if(self.debug):
            print("Socket Connected")
        # Variable to control writing and reading frequency
        self.inLoopSleepTime = 0.04
        self.outLoopSleepTime = 0
        # Variable to control writing loop i.e. if we want to write data or not
        self.writeLoop = False
        self.readLoop = False
        # list of all type of Payload of all the OUT Packets to be requested
        self.outServices = {"MSP_ATTITUDE":108,"MSP_ALTITUDE":109,"MSP_RAW_IMU":102,"MSP_ACC_TRIM":240,"MSP_RC":105,"MSP_COMMAND":124}
        # list to store MSP messages of the OUT Packets we request
        self.requestMSPPackets = []
        # variable to control buffer size of OUT loop 
        self.outBufferSize = 64
        # Dictionary of all the parameters we write to the pluto drone
        self.paramsSet = {"Roll":1500,"Pitch":1500,"Throttle":1500,"Yaw":1500,"Aux1":1500,"Aux2":1500,"Aux3":1500,"Aux4":1000,"currentCommand":0,"trimPitch":0,"trimRoll":0}
        # Dictionary of all the parameters we read from the pluto drone
        self.paramsReceived = {"timeOfLastUpdate":-1,"Roll":-1,"Pitch":-1,"Yaw":-1,"Altitude":-1,"Vario":-1,"AccX":-1,"AccY":-1,"AccZ":-1,"GyroX":-1,"GyroY":-1,"GyroZ":-1,"MagX":-1,"MagY":-1,"MagZ":-1,"trimRoll":-1,"trimPitch":-1,"rcRoll":-1,"rcPitch":-1, "rcYaw":-1, "rcThrottle" :-1, "rcAUX1":-1, "rcAUX2":-1, "rcAUX3":-1, "rcAUX4":-1,"currentCommand":-1,"commandStatus":-1 }
    
    def arm(self):
        """
        Member Function to Arm the drone.
        """
        self.paramsSet["Roll"] = 1500
        self.paramsSet["Pitch"] = 1500
        self.paramsSet["Throttle"] = 1000
        self.paramsSet["Aux4"] = 1500
        time.sleep(self.waitTime)
    
    def boxArm(self):
        """
        Member Function to BoxArm the drone. 
        """
        self.paramsSet["Roll"] = 1500
        self.paramsSet["Pitch"] = 1500
        self.paramsSet["Throttle"] = 1500
        self.paramsSet["Aux4"] = 1500
        time.sleep(self.waitTime)
    
    def disArm(self):
        """
        Member Function to Disarm the drone.
        """ 
        self.paramsSet["Throttle"] = 1300
        self.paramsSet["Aux4"] = 1000
        time.sleep(self.waitTime)
    
    def forward(self):
        """
        Member Function to increase pitch so that the drone moves forward.
        """
        self.paramsSet["Pitch"] = 1600
        time.sleep(self.waitTime)
        
    def backward(self):
        """
        Member function to decrease pitch so that the drone moves backward.
        """
        self.paramsSet["Pitch"] = 1400
        time.sleep(self.waitTime)
        
    def left(self):
        """
        Member Function to decrease roll so that the drone moves towards left.
        """
        self.paramsSet["Roll"] = 1400
        time.sleep(self.waitTime)
        
    def right(self):
        """
        Member Function to increase roll so that the drone moves towards right.
        """
        self.paramsSet["Roll"] = 1600
        time.sleep(self.waitTime)
        
    def leftYaw(self):
        """
        Member Function to decrease yaw so that the drone rotates left.
        """
        self.paramsSet["Yaw"] = 1200
        time.sleep(self.waitTime)
    
    def rightYaw(self):
        """
        Member Function to decrease yaw so that the drone rotates right.
        """
        self.paramsSet["Yaw"] = 1800
        time.sleep(self.waitTime)
    
    def reset(self):
        """
        Member Function to set all parameters to set to default values.
        """
        self.paramsSet["Roll"] = 1500
        self.paramsSet["Pitch"] = 1500
        self.paramsSet["Throttle"] = 1500
        self.paramsSet["Yaw"] = 1500
        self.paramsSet["currentCommand"] = 0
        time.sleep(self.waitTime)
    
    def increaseHeight(self):
        """
        Member Function to increase throttle so that height of drone is increased.
        """
        self.paramsSet["Throttle"] = 1800
        time.sleep(self.waitTime)
    
    def lowThrottle(self):
        """
        Member Function to provide a low throttle, generally used for hardware testing.
        """
        self.paramsSet["Throttle"] = 901
        time.sleep(self.waitTime)
    
    def decreaseHeight(self):
        """
        Member Function to decrease throttle so that height of drone is decreased.
        """
        self.paramsSet["Throttle"] = 1300
        time.sleep(self.waitTime)
    
    def takeOff(self):
        """
        Member Function to set command as TakeOff.
        """
        self.disArm()
        self.boxArm()
        self.paramsSet["currentCommand"] = 1
        time.sleep(self.waitTime)
    
    def land(self):
        """
        Member Function to set command as Land.
        """
        self.reset()
        self.paramsSet["currentCommand"] = 2
        # Giving extra 2 seconds to land
        time.sleep(self.waitTime+3)
        self.disArm()
    
    def backFlip(self):
        """
        Member Function to set command as Back Flip.
        """
        self.paramsSet["currentCommand"] = 3
        time.sleep(self.waitTime)
    
    # function to generate MSP Packets for all OUT services 
    def getAllRequestMsgs(self):
        """
        Member Function to generate MSP Packets for all Out Packets mentioned in OUT services.
        """
        for service in self.outServices.values():
            self.requestMSPPackets.append(MSPPacket().getInMsgRequest(0,service,[]))

    """
    Target of the Writing Thread
    ALl writing to drone takes place in this function untill we set sendData to false
    """
    def write(self, commandLock):
        """
        Member Function which is the target of the Writing Thread.
        All writing to the drone takes place in this function until we set sendData to False
        
        Before entering the loop we set the Roll and Pitch Trim, as it is to be done only once in the starting.
        In the loop we keep sending the SET_RAW_RC commands, we check if SET_COMMAND is not 0 then we send that also.
        In every iteration, we also request for all the messages we want as OUT Packets.
        """
        self.writeLoop = True
        self.getAllRequestMsgs()
        msgLen = 4
        typeOfPayload = 239
        msgData = []
        msgData.extend(getBytes(toUnsigned(self.paramsSet["trimPitch"])))
        msgData.extend(getBytes(toUnsigned(self.paramsSet["trimRoll"])))
        msgToBeSent = MSPPacket().getInMsgRequest(msgLen,typeOfPayload,msgData)
        if self.debug :
            print(msgToBeSent)
        self.mySocket.send(bytearray(msgToBeSent))
        while(self.writeLoop):
            # Sending MSP_SET_RAW_RC type message
            start_time = time.time()
            msgLen = 16
            typeOfPayload = 200
            msgData = []
            commandLock.acquire()
            msgData.extend(getBytes(self.paramsSet["Roll"]))
            msgData.extend(getBytes(self.paramsSet["Pitch"]))
            msgData.extend(getBytes(self.paramsSet["Throttle"]))
            msgData.extend(getBytes(self.paramsSet["Yaw"]))
            msgData.extend(getBytes(self.paramsSet["Aux1"]))
            msgData.extend(getBytes(self.paramsSet["Aux2"]))
            msgData.extend(getBytes(self.paramsSet["Aux3"]))
            msgData.extend(getBytes(self.paramsSet["Aux4"]))
            commandLock.release()
            
            msgToBeSent = MSPPacket().getInMsgRequest(msgLen,typeOfPayload,msgData)
            if self.debug :
                print("message: ",msgToBeSent)
            self.mySocket.send(bytearray(msgToBeSent))
            # Sending MSP_SET_COMMAND type message
            commandLock.acquire()
            if self.paramsSet["currentCommand"]!=0:
                msgLen = 2
                typeOfPayload = 217
                msgData = []
                msgData.extend(getBytes(self.paramsSet["currentCommand"]))
                self.paramsSet["currentCommand"]=0
                commandLock.release()
                msgToBeSent = MSPPacket().getInMsgRequest(msgLen,typeOfPayload,msgData)
                if self.debug :
                    print(msgToBeSent)
                self.mySocket.send(bytearray(msgToBeSent))
            else:
                commandLock.release()

            # sending MSP request messages for OUT Packets
            for request in self.requestMSPPackets:
                # print(request)
                self.mySocket.send(bytearray(request))
            # sleep to control writing frequency
            end_time = time.time()
            loop_time = end_time - start_time
            if loop_time > self.inLoopSleepTime:
                print("ERRORRRR : Writing at less than 25 Hz")
            else:
                time.sleep(self.inLoopSleepTime - loop_time)

    """
    Target function to the reading thread
    All the data sent by the drone is received in this function
    """
    def read(self,IMUQueue,imuLock):
        """
        Member Function which is target function to the Reading Thread.
        All the data sent by the drone is received in this function.
        
        In the loop we wait for 2 seconds max, if we do not receive any data on the socket then we break out of the loop.
        While receiving we append that message at the end of the currently existing buffer.
        After this the current buffer is again and again processed and the parameter received are updated until the buffer can no longer be processed.
        
        """
        # buffer to hold the data sent by drone
        buff = []
        self.readLoop = True
        # s = time.time()
        # # i = 0
        # sum = 0
        # Always reading values and updating the buffer within the readThread utill readLoop is set to false
        while(self.readLoop):
            # print("readloop")
            start_time = time.time()
            # Time out after 2 seconds of not getting data
            ready = select.select([self.mySocket],[],[],2) 
            if not ready[0]:
                break
            
            buff = self.receiveMSPresponse(buff) 
            imuLock.acquire()
            IMUQueue.append(self.paramsReceived)
            imuLock.release()
            if self.debug:
                self.printParams()
                print(f"Read at {time.time()-start_time} s")
    
    # function to print all the parameters
    def printParams(self):
        """
        Member Function to Print all the Parameters.
        """
        print("ParamsSet: ",self.paramsSet)
        print("ParamsReceived: ",self.paramsReceived)
        # print(list(self.paramsReceived.values()))
    
    # function to update the received parameters
    def updateParams(self,out,idx):
        """
        Member Function to update the received parameters.
        :param out: Decoded Values
        :type out: List
        :param idx: value of type of payload 
        :type idx: int
        """
        self.paramsReceived["timeOfLastUpdate"] = time.time()
        if idx==self.outServices["MSP_ATTITUDE"]: #MSP_ATTITUDE
            self.paramsReceived["Roll"]=out[0]
            self.paramsReceived["Pitch"]=out[1]
            self.paramsReceived["Yaw"]=out[2]
        elif idx==self.outServices["MSP_ALTITUDE"]: #MSP_ALTITUDE
            self.paramsReceived["Altitude"]=out[0]
            self.paramsReceived["Vario"]=out[1]
        elif idx==self.outServices["MSP_RAW_IMU"]: #MSP_RAW_IMU
            self.paramsReceived["AccX"]=out[0]
            self.paramsReceived["AccY"]=out[1]
            self.paramsReceived["AccZ"]=out[2]
            self.paramsReceived["GyroX"]=out[3]
            self.paramsReceived["GyroY"]=out[4]
            self.paramsReceived["GyroZ"]=out[5]
            self.paramsReceived["MagX"]=out[6]
            self.paramsReceived["MagY"]=out[7]
            self.paramsReceived["MagZ"]=out[8]
        elif idx==self.outServices["MSP_ACC_TRIM"]:
            self.paramsReceived["trimPitch"]=out[0]
            self.paramsReceived["trimRoll"]=out[1]
        elif idx==self.outServices["MSP_RC"]:
            self.paramsReceived["rcRoll"]=out[0]
            self.paramsReceived["rcPitch"]=out[1]
            self.paramsReceived["rcYaw"]=out[2]
            self.paramsReceived["rcThrottle"]=out[3]
            self.paramsReceived["rcAUX1"]=out[4]
            self.paramsReceived["rcAUX2"]=out[5]
            self.paramsReceived["rcAUX3"]=out[6]
            self.paramsReceived["rcAUX4"]=out[7]         
        elif idx==self.outServices["MSP_COMMAND"]:
            self.paramsReceived["currentCommand"] = out[0]
            self.paramsReceived["commandStatus"] = out[1]
        
        if self.debug:
            self.printParams()
    
    # Reads data and appends it at the end of buffer 
    def updateBuffer(self,buff):
        """
        Member Function to read data and appends it at the end of buffer. 
        """
        arr = self.mySocket.recv(self.outBufferSize)
        buff.extend(list(arr))
        if(self.debug):
            print("buff: ",buff)
        return buff
    

    def processBuffer(self,buff,funDebug = False):
        """
        This function processes the current buffer and return the list of decoded values as out list, 
        the OUT Packet payload corresponding to which the values are and the remaining buffer.
        Returns empty out list and 0 as payload if buffer does not contain the full message
        """
        if self.debug or funDebug:
            print(buff)
        # If length of buffer is less than 5 then it cannot conatin a message, so return buffer as it is
        if len(buff)<5:
            return [],0,buff
        
        # Checking the header and direction of message, if it does not match we report an error or handle accordingly
        headerArrayOut = [36,77,62]
        for i in range(3):
            if buff[i]!=headerArrayOut[i]:
                if i==2:
                    if buff[i]==33:
                        # if self.debug:
                        print("Error sent in out packet....!!!!!",buff)
                        return
                    else:
                        buff = buff[2:]
                        return [],0,buff
                else:
                    index = -1
                    for i in range(len(buff)):
                        if buff[i]==headerArrayOut[0] and i+1!=len(buff):
                            if buff[i+1]==headerArrayOut[1]:
                                index = i
                    if index!=-1 :
                        buff = buff[index:]
                        if self.debug:
                            print("garbage received (even header does not match)..!! IGNORED",buff)
                        return [],0,buff
                    else:
                        if self.debug:
                            print("Error decoding buffer could not be resolved (direction does not make sense)...!!! IGNORED",buff)
                        return [],0,buff
        
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
                    out.append(getSignedDec(buff[i+5],buff[i+6]))
                out[0] /= 10
                out[1] /= 10
                if out[2]>180:
                    out[2] = out[2]-360
            elif idx==self.outServices["MSP_ALTITUDE"]:
                lsb16 = getDec(buff[5],buff[6])
                msb16 = getDec(buff[7],buff[8])
                # dividing by 100 to convert to meters and meter-per-second
                out.append(getDec(lsb16,msb16,256*256)/100)
                out.append(getSignedDec(buff[9],buff[10])/100)
            elif idx==self.outServices["MSP_RAW_IMU"]:
                for i in range(0,msgLen,2):
                    out.append(getSignedDec(buff[i+5],buff[i+6]))
                for i in range(3,6):
                    out[i] =out[i]/8
                for i in range(6,9):
                    out[i] = out[i]/3
            elif idx==self.outServices["MSP_ACC_TRIM"]:
                out.append(getDec(buff[5],buff[6]))
                out.append(getDec(buff[7],buff[8]))
            elif idx==self.outServices["MSP_RC"]:
                for i in range(0,msgLen,2):
                    out.append(getSignedDec(buff[i+5],buff[i+6])*10)
            elif idx==self.outServices["MSP_COMMAND"]:
                out.append(getDec(buff[5],buff[6]))
                out.append(buff[7])
            
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
                index = -1
                for i in range(len(buff)):
                    if buff[i]==headerArrayOut[0] and i+1!=len(buff):
                        if buff[i+1]==headerArrayOut[1]:
                            index = i
                if index!=-1 :
                    buff = buff[index:]
                    if self.debug:
                        print("Error in decoding the buffer....!!!! -> IGNORED",buff)
                    return [],0,buff
                else:
                    if self.debug:
                        print("Error decoding buffer could not be resolved (checksum does not match)...!!! IGNORED",buff)
                    return [],0,buff
        else:
            # return buffer as it is if complete message is not present
            return [],0,buff
    
    # function to receive data and process it to return decoded values and updated buffer
    def receiveMSPresponse(self,buff): 
        """
        Member Function to receive data and process it to return decoded values and updated buffer.
        """        
        buff = self.updateBuffer(buff)
        i=len(buff)//3
        while len(buff)>5:    
            i-=1
            if i<0:
                break
            out,idx,buff = self.processBuffer(buff) 
            self.updateParams(out,idx) 
        return buff

    # function to disconnect with the communication
    def disconnect(self):
        """
        Member Function to disconnect communication and stop read and writing loops.
        """
        self.readLoop = False
        self.writeLoop = False
        self.mySocket.close()
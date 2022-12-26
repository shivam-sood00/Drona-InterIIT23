import socket,time,math

class MSP_SET_RAW_RC:
    def __init__(self,mySocket,debug=False):
        self.mySocket=mySocket
        #header(2 bytes), 60 - to the controller, 62 - from the contr.
        headerArray=bytearray([36,77,60])
        self.valueArray=bytearray([])
        self.debug = debug
        roll=1500                    
        pitch=1500                 
        throttle=1800 
        yaw=1500                      
        aux1=1500
        #Dev mode 
        # aux2=1000
        aux2=1500
        aux3=1500
        aux4=1500      
        self.valueArray.extend(headerArray)
        self.valueArray.append(16) #MSG LENGTH
        self.valueArray.append(200) #MSG TYPE FOR RAW_RC
        #Initialising in terms of bytes (roll,pitch...)
        # (l)
        # self.valueArray.extend([220,5]) 
        # self.valueArray.extend([220,5])
        # self.valueArray.extend([220,5])
        # self.valueArray.extend([220,5])
        # self.valueArray.extend([176,4])
        # self.valueArray.extend([220,5])
        # self.valueArray.extend([220,5])
        # self.valueArray.extend([176,4])
        # import pdb;pdb.set_trace()
        self.valueArray.extend(list(self.getBytes(roll)))
        self.valueArray.extend(list(self.getBytes(pitch)))
        self.valueArray.extend(list(self.getBytes(throttle)))
        self.valueArray.extend(list(self.getBytes(yaw)))
        self.valueArray.extend(list(self.getBytes(aux1)))
        self.valueArray.extend(list(self.getBytes(aux2)))
        self.valueArray.extend(list(self.getBytes(aux3)))
        self.valueArray.extend(list(self.getBytes(aux4)))
        
        #Checksum
        self.valueArray.append(234)
        self.Array=self.valueArray[:]
        if(self.debug):
            print(self.Array)
        self.isConnected=False
        
    def changeCRC(self):
        self.CRCArray=self.Array[3:-1]
        self.CRCValue=0
        for d in self.CRCArray:
            self.CRCValue= self.CRCValue^d
        return self.CRCValue
    
    def getBytes(self,value): 
        self.LSB=value % 256
        self.MSB=math.floor(value/256)
        return bytearray([self.LSB,self.MSB])

##    def connect(self):
##        self.isConnected=True
##        if(self.debug):        
##            print ("Connected to Drone")            
##        self.sendPacket(self.Array)
        
        

    def arm(self):           
        self.Array[19]=220
        self.Array[20]=5
        Val=self.changeCRC()
        self.Array[21]=Val
        if(self.debug):
            print("Armed")
        self.sendPacket(self.Array)
            
            

##        else:
##            self.Array[21]=0
##            if(self.debug):
##                print("Not Connected")
##            self.sendPacket(self.Array)
            
            

    
    def disarm(self):
        #AUX-4 = 1500
        self.Array[19]=176
        self.Array[20]=4
        Val=self.changeCRC()
        self.Array[21]=Val
        self.sendPacket(self.Array)
        if(self.debug):
            print("Dirmed")
        
        
    
    def setThrottle(self,value):            
        arr=bytearray([])
        arr.extend(self.getBytes(value))
        self.Array[9]=arr[0]
        self.Array[10]=arr[1]
        #aux 3 = 1800 for throttle 
        # self.Array[17]=8
        # self.Array[18]=7
        Val=self.changeCRC()
        self.Array[21]=Val
        if(self.debug):
            print("Throttle set to ",value)
        print(list(self.Array))
        self.sendPacket(self.Array)
        
    
    def setRoll(self,value):                  
        arr=bytearray([])
        arr.extend(self.getBytes(value))
        self.Array[5]=arr[0]
        self.Array[6]=arr[1]
        Val=self.changeCRC()
        self.Array[21]=Val
        if(self.debug):
            print("Roll set to ",value)
        self.sendPacket(self.Array)
        
        
    
    def setPitch(self,value):                
        arr=bytearray([])
        arr.extend(self.getBytes(value))
        self.Array[7]=arr[0]
        self.Array[8]=arr[1]
        Val=self.changeCRC()
        self.Array[21]=Val
        self.sendPacket(self.Array)
        
        

    def setYaw(self,value):              
        arr=bytearray([])
        arr.extend(self.getBytes(value))
        self.Array[11]=arr[0]
        self.Array[12]=arr[1]
        Val=self.changeCRC()
        self.Array[21]=Val
        self.sendPacket(self.Array)
        
        

    def sendPacket(self,lValueArray):
        self.mySocket.send(lValueArray)

    def recieveResponse(self):
        return self.mySocket.recv(self.BUFFER_SIZE)


class MSP_SET_COMMAND:
    def __init__(self,mySocket,debug=False):
        self.mySocket = mySocket
        self.debug = debug
        headerArray=bytearray([36,77,60])
        self.valueArray=bytearray([])    
        self.valueArray.extend(headerArray)
        self.valueArray.append(2)
        self.valueArray.append(217)
        #default??
        self.valueArray.extend([0,0])
        self.valueArray.append(234)
        self.Array=self.valueArray[:]
        if(self.debug):
            print(self.Array)
        self.isConnected=False
    
    def changeCRC(self):
        self.CRCArray=self.Array[3:-1]
        self.CRCValue=0
        for d in self.CRCArray:
            self.CRCValue= self.CRCValue^d
        return self.CRCValue
    
    def getBytes(self,value): 
        self.LSB=value % 256
        self.MSB=math.floor(value/256)
        return bytearray([self.LSB,self.MSB])
    
    def takeOff(self):
        arr=bytearray([])
        arr.extend(self.getBytes(1))
        self.Array[5]=arr[0]
        self.Array[6]=arr[1]
        Val=self.changeCRC()
        self.Array[7]=Val
        if(self.debug):
            print("Taking Off")
        self.sendPacket(self.Array)
    
    def land(self):
        arr=bytearray([])
        arr.extend(self.getBytes(2))
        self.Array[5]=arr[0]
        self.Array[6]=arr[1]
        Val=self.changeCRC()
        self.Array[7]=Val
        if(self.debug):
            print("Landing")
        self.sendPacket(self.Array)

    def backFlip(self):
        arr=bytearray([])
        arr.extend(self.getBytes(3))
        self.Array[5]=arr[0]
        self.Array[6]=arr[1]
        Val=self.changeCRC()
        self.Array[7]=Val
        if(self.debug):
            print("Back Flip")
        self.sendPacket(self.Array)

    def frontFlip(self):
        arr=bytearray([])
        arr.extend(self.getBytes(4))
        self.Array[5]=arr[0]
        self.Array[6]=arr[1]
        Val=self.changeCRC()
        self.Array[7]=Val
        if(self.debug):
            print("Front flip")
        self.sendPacket(self.Array)
    
    def rightFlip(self):
        arr=bytearray([])
        arr.extend(self.getBytes(5))
        self.Array[5]=arr[0]
        self.Array[6]=arr[1]
        Val=self.changeCRC()
        self.Array[7]=Val
        if(self.debug):
            print("Right Flip")
        self.sendPacket(self.Array)
        
    def leftFlip(self):
        arr=bytearray([])
        arr.extend(self.getBytes(6))
        self.Array[5]=arr[0]
        self.Array[6]=arr[1]
        Val=self.changeCRC()
        self.Array[7]=Val
        if(self.debug):
            print("Left Flip")
        self.sendPacket(self.Array)

    def sendPacket(self,lValueArray):
        self.mySocket.send(lValueArray)

    def recieveResponse(self):
        return self.mySocket.recv(self.BUFFER_SIZE)
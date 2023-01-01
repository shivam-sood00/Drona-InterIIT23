"""
Control Your Drone!
---------------------------
Moving around:
    u    i    o
    j    k    l
    n    m    ,


spacebar : arm or disarm
w : increase height
s : decrease height
q : take off
e : land
a : yaw left
d : yaw right
t : auto pilot on/off
Up arrow : go forward
Down arrow : go backward
Left arrow : go left
Right arrow : go right

CTRL+C to quit

"""

import threading,plutoComms,time


inputVal = None

def inputFun():
    global inputVal
    while(inputVal!="p"):
        inputVal = input("enter key: ")

if __name__=="__main__":
    comms = plutoComms.COMMS(debug=False)
    # toggle = False
    readThread = threading.Thread(target=comms.read)
    writeThread = threading.Thread(target=comms.write)
    writeThread.start()
    readThread.start()
    
    inputVal = "o"
    while(inputVal!="p"):
        inputVal = input("enter key: ")
        print("inputVal: ",inputVal)
        if inputVal=="q":
            comms.takeOff()
        elif inputVal==",":
            # toggle = not toggle
            comms.arm()
        elif inputVal==".":
            comms.disArm()
        elif inputVal=="/":
            comms.boxArm()
        elif inputVal=="e":
            comms.land()
        elif inputVal=="w":
            comms.increaseHeight()
        elif inputVal=="s":
            comms.decreaseHeight()
        elif inputVal=="a":
            comms.leftYaw()
        elif inputVal=="d":
            comms.rightYaw()
        elif inputVal=="u":
            comms.forward()
        elif inputVal=="j":
            comms.backward()
        elif inputVal=="h":
            comms.left()
        elif inputVal=="k":
            comms.right()
        elif inputVal=="b":
            comms.backFlip()
        elif inputVal=="r":
            comms.reset()
        # q = cv2.waitKey(1)
    comms.disconnect()   
    writeThread.join()
    readThread.join()
        # if q!=-1:
        #     print(q)
        #     inputVal=q

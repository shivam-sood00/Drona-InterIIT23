
# a file named "geek", will be opened with the reading mode.
import time
file = open('debug.txt', 'r')
# This will print every line one by one in the file
x=[]
y=[]
z=[]
errX=[]
errY=[]
errZ=[]
roll=[]
pitch=[]
throttle=[]
yaw=[]
stateYaw=[]
for each in file:
    if each[0]=="L":
        break
    q=0
    msg=[]
    sign=0
    decimal = 0
    dig = 0
    for i in range(len(each)):
        if each[i]==" ":
            if decimal:
                q = q/10**dig
            if sign:
                q = -q
            msg.append(q)
            q=0
            dig=0
            decimal=0
            sign=0
        elif each[i]=="-":
            sign=1
        elif each[i]==".":
            decimal=1
            dig = 0
        elif each[i]=="\n":
            if decimal:
                q = q/10**dig
            if sign:
                q = -q
            msg.append(q)
            q=0
            dig=0
            decimal=0
            sign=0
        elif each[i]=="[" or each[i]=="]":
            continue
        else:
            q = q*10 + int(each[i])
            dig += 1
    
    # msg.append(q)
    
    print(msg)
    x.append(msg[0])
    y.append(msg[1])
    z.append(msg[2])
    
    roll.append(msg[3])
    pitch.append(msg[4])
    yaw.append(msg[5])
    throttle.append(msg[6])
    
    errY.append(msg[7])
    errX.append(msg[8])
    errZ.append(msg[9])
    
    stateYaw.append(msg[10])

import matplotlib.pyplot as plt



# print(roll)

plt.plot(roll)
plt.savefig(f"graphs/roll{time.time()}.png")
plt.clf()

plt.plot(pitch)
plt.savefig(f"graphs/pitch{time.time()}.png")
plt.clf()

plt.plot(yaw)
plt.savefig(f"graphs/yaw{time.time()}.png")
plt.clf()

plt.plot(throttle)
plt.savefig(f"graphs/throttle{time.time()}.png")
plt.clf()

plt.plot(x)
plt.savefig(f"graphs/x{time.time()}.png")
plt.clf()

plt.plot(y)
plt.savefig(f"graphs/y{time.time()}.png")
plt.clf()

plt.plot(z)
plt.savefig(f"graphs/z{time.time()}.png")
plt.clf()


plt.plot(errX)
plt.savefig(f"graphs/errX{time.time()}.png")
plt.clf()

plt.plot(errY)
plt.savefig(f"graphs/errY{time.time()}.png")
plt.clf()

plt.plot(errZ)
plt.savefig(f"graphs/errZ{time.time()}.png")
plt.clf()

plt.plot(stateYaw)
plt.savefig(f"graphs/stateYaw{time.time()}.png")
plt.clf()
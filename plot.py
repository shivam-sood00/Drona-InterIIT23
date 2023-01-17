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
statePitch=[]
stateRoll=[]
for each in file:
    if each[0]=="L":
        break
    elif each[0]>="A" and each[0]<="Z" or each[0]>="a" and each[0]<="z":
        continue
    q=0
    msg=[]
    sign=0
    decimal = 0
    dig = 0
    eva = 0
    for i in range(len(each)):
        if each[i]==" ":
            if decimal:
                q = q/10**dig
            if sign:
                q = -q
            if eva!=0:
                q = q/10**eva
            msg.append(q)
            q=0
            dig=0
            decimal=0
            sign=0
            eva=0
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
        elif each[i]=='e':
            i = i+3
            eva = int(each[i])
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
    stateRoll.append(msg[11])
    statePitch.append(msg[12])

import matplotlib.pyplot as plt

from datetime import datetime
from os.path import join
from configparser import ConfigParser
from os import makedirs
# import yaml


folder_name = datetime.now().strftime("%m-%d, %H:%M:%S")
folder_name = join("graphs",folder_name)
makedirs(folder_name)

# config = yaml.load(open("controls/config.yaml"), Loader=yaml.FullLoader)

# with open(join(folder_name,"config.yaml"),'w') as outfile:
#     yaml.dump(config,outfile,default_flow_style=False)
config = ConfigParser()
config.read("controls/droneData.ini")
with open(join(folder_name,"DroneData.ini"), 'w') as configfile:
    config.write(configfile)


plt.plot(roll)
plt.savefig(f"{folder_name}/roll.png")
plt.clf()

plt.plot(pitch)
plt.savefig(f"{folder_name}/pitch.png")
plt.clf()

plt.plot(yaw)
plt.savefig(f"{folder_name}/yaw.png")
plt.clf()

plt.plot(throttle)
plt.savefig(f"{folder_name}/throttle.png")
plt.clf()

plt.plot(x)
plt.savefig(f"{folder_name}/x.png")
plt.clf()

plt.plot(y)
plt.savefig(f"{folder_name}/y.png")
plt.clf()

plt.plot(z)
plt.savefig(f"{folder_name}/z.png")
plt.clf()


plt.plot(errX)
plt.savefig(f"{folder_name}/errX.png")
plt.clf()

plt.plot(errY)
plt.savefig(f"{folder_name}/errY.png")
plt.clf()

plt.plot(errZ)
plt.savefig(f"{folder_name}/errZ.png")
plt.clf()

plt.plot(stateYaw)
plt.savefig(f"{folder_name}/stateYaw.png")
plt.clf()

plt.plot(stateRoll)
plt.savefig(f"{folder_name}/stateRoll.png")
plt.clf()

plt.plot(statePitch)
plt.savefig(f"{folder_name}/statePitch.png")
plt.clf()
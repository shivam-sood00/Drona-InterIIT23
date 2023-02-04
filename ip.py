import os
devices = []
ip_s = ""
ip = []
start=0;end=0
for device in os.popen('arp -a'): 
    devices.append(device)
    for i in range(len(device)):
        if(device[i]=='('):
            start = i+1
        if(device[i]==')'):
            end = i
    ip.append(device[start:end])


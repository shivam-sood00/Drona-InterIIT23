
# a file named "geek", will be opened with the reading mode.
file = open('takeoffandland.txt', 'r')
# This will print every line one by one in the file
prevMsg=[]
for each in file:
    # print(each,int(each[0]))
    q = 0
    msg = []
    sign=0
    for i in range(len(each)):
        if each[i]==" ":
            if sign:
                q = 256-q
            msg.append(q)
            q=0
            sign=0
        elif each[i]=="-":
            sign=1
        elif each[i]=="\n":
            continue
        else:
            q = q*10 + int(each[i])
    # print(msg)
    if msg[3]!=0:
        # print(msg)
        if prevMsg!=msg:
            prevMsg=msg
            print(msg)
            vals=[]
            if msg[3]!=16:
                continue
            for i in range(5,len(msg)-1,2):
                vals.append(msg[i]+msg[i+1]*256)
            print(vals)
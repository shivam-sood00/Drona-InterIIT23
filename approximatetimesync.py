def flag(q1,q2,threshold = 0.05):

    if len(q1) == 0 and len(q2) == 0:
        return 3
    elif len(q1) == 0:
        return 2
    elif len(q2) == 0:
        return 1
    else:
        timestamp1 = q1[-1]["timeOfLastUpdate"]
        timestamp2 = q2[-1][0]

        if abs(timestamp1-timestamp2)>threshold:
            if timestamp1 > timestamp2:
                return 2
            else:
                return 1
        return 0

def time_sync(q1, q2):
    latetime = None
    v1 = None
    v2 = None

    k = flag(q1,q2)
    if k==0:
        v1 = q1[-1]
        v2 = q2[-1]
        q1.clear()
        q2.clear()

    elif k==1:
        v1 = q1[-1]
        q1.clear()

    elif k==2:
        v2 = q2[-1]
        q2.clear()

    sensorData = {"imu":v1, "camera":v2}
    return k,sensorData    




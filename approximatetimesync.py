def flag(q1,q2):
    if q1.empty() and q2.empty():
        return 3
    elif q1.empty():
        return 2
    elif q2.empty():
        return 1
    else:
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
    sensorData = {"imu":v1, "camera":v2}
    return k,sensorData    




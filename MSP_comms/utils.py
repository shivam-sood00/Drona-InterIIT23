import math

def getBytes(value): 
    LSB=value % 256
    MSB=math.floor(value/256)
    return [LSB,MSB]   

def getDec(lsb,msb,base=256):
    return lsb+base*msb

def getSignedDec(lsb,msb):
    a = getDec(lsb,msb)
    if msb>127:
        a = -2**16+a
    return a

def getCRC(arr,debug=False):
    CRCArray=arr[3:]
    if debug:
        print("CRC arrary = ",list(CRCArray))
    CRCValue=0
    for d in CRCArray:
        CRCValue= CRCValue^d
    return CRCValue
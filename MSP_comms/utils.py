import math

def getBytes(value): 
    """
    Function to return 8-bit LSB and MSB as a list of the given value
    """
    LSB=value % 256
    MSB=math.floor(value/256)
    return [LSB,MSB]   

def getDec(lsb,msb,base=256):
    """
    Function to return back the unsigned number given LSB and MSB. 
    The base is default at 256 but can be passed as per requirement
    """
    return lsb+base*msb

def getSignedDec(lsb,msb):
    """
    Function to return signed number given the LSB and MSB
    """
    a = getDec(lsb,msb)
    if msb>127:
        a = -2**16+a
    return a
class arucoVelocity:
    def __init__(self, X, Y, time) -> None:
        self.prevTime = time
        self.prevX = X
        self.prevY = Y

    def getVelocity(self,X,Y,time):
        velX = (X - self.prevX)/(time - self.prevTime)
        velY = (Y - self.prevY)/(time - self.prevTime)
        self.prevTime = time
        self.prevX = X
        self.prevY = Y
        return [velX,velY]
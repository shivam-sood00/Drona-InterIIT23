import numpy as np
class movingAverage:
    def __init__(self,no_of_data,horizon) -> None:
        self.horizon = horizon
        self.data_fr_ma = np.zeros((no_of_data, horizon))
        self.no_of_data = no_of_data
        self.counter = 0
    
    def getAverage(self,data):
        self.data_fr_ma[:,0:self.horizon-1] = self.data_fr_ma[:,1:self.horizon]
        
        for i in range(self.no_of_data):
            self.data_fr_ma[i,self.horizon-1] = data[i]

        estimated = np.average(self.data_fr_ma, axis=1) 

        if self.counter < self.horizon:
            self.counter += 1
            return data
        else:
            return estimated

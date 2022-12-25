#!/usr/bin/env python3
import numpy as np
import cv2
from cv2 import aruco
import math


# Reference:https://aliyasineser.medium.com/calculation-relative-positions-of-aruco-markers-eee9cc4036e3

class Transform:

    def __init__(self,TvecA,RvecA,TvecB,RvecB):
        self.TvecA = TvecA
        self.TvecB = TvecB
        self.RvecA = RvecA
        self.RvecB = RvecB
    # A to B Transform denoted by A -> B
    # Goes from Camera -> Tag to Tag -> Camera   
    def inversePerspective(rvec, tvec):
        R, _ = cv2.Rodrigues(rvec)
        R = np.matrix(R).T
        invTvec = np.dot(R, np.matrix(-tvec))
        invRvec, _ = cv2.Rodrigues(R)
        return invRvec, invTvec
    
    def relativePosition(self):
        self.RvecA, self.TvecA = self.RvecA.reshape((3, 1)), self.TvecA.reshape((3, 1))
        self.RvecB, self.TvecB = self.RvecB.reshape((3, 1)), self.TvecB.reshape((3, 1))
        
        invRvec, invTvec = self.inversePerspective(self.RvecB, self.TvecB)
        
        # Compose Tag B -> Camera and Camera -> Tag A to get Tag B -> Tag A
        info = cv2.composeRT(self.RvecA, self.TvecA, invRvec, invTvec)
        composedRvec, composedTvec = info[0], info[1]
        composedRvec = composedRvec.reshape((3, 1))
        composedTvec = composedTvec.reshape((3, 1))

        # Tag B -> Tag A
        return composedRvec, composedTvec

from roboticstoolbox import *
import matplotlib.pyplot as plt
import roboticstoolbox as rtb
import math
import numpy as np
from spatialmath.base import *
from spatialmath import *
import matplotlib as plt




class Robot5R(DHRobot):
    def __init__(self):
        l1 = 5.475/100;
        l2 = 8.0/100;
        l3 = 5.209/100;
        l4 = 3.694/100;
        l5 = 10.486/100;
        R = []
        R.append(RevoluteDH(d=l1, a=0, alpha=math.pi/2, offset=0))
        R.append(RevoluteDH(d=0, a=l2, alpha=0, offset=0))
        R.append(RevoluteDH(d=0, a=0, alpha=math.pi/2, offset=math.pi/2))
        R.append(RevoluteDH(d=l3+l4, a=0, alpha=-math.pi/2, offset=0))
        R.append(RevoluteDH(d=0, a=l5, alpha=0, offset=-math.pi/2))
        super().__init__(R, name="5R", keywords=("Angular5R",))
        
    def rotarX(self, a, theta):
        if a == 'rad':
            r = [[1, 0, 0],[0, np.cos(theta), -np.sin(theta)],[0, np.sin(theta), np.cos(theta)]]
            r = np.round(r,5)
            print(r)
        elif a == 'deg':
            r = [[1, 0, 0],[0, np.cos(np.deg2rad(theta)), -np.sin(np.deg2rad(theta))],[0, np.sin(np.deg2rad(theta)), np.cos(np.deg2rad(theta))]]
            r = np.round(r,5)
            print(r)
        
    def rotarY(self, a, theta):
        if a == 'rad':
            r = [[np.cos(theta), 0, np.sin(theta)],[0, 1, 0],[-np.sin(theta), 0, np.cos(theta)]]
            #r = np.round(r,5)
            print(r)
        elif a == 'deg':
            r = [[np.cos(np.deg2rad(theta)), 0, np.sin(np.deg2rad(theta))],[0, 1, 0],[-np.sin(np.deg2rad(theta)), 0, np.cos(np.deg2rad(theta))]]
            #r = np.round(r,5)
            print(r)
    def rotarZ(self, a, theta):
        if a == 'rad':
            r = [[np.cos(theta), -np.sin(theta), 0],[np.sin(theta), np.cos(theta), 0],[0, 0, 1]]
            #r = np.round(r,5)
            print(r)
        elif a == 'deg':
            r = [[np.cos(np.deg2rad(theta)), -np.sin(np.deg2rad(theta)), 0],[np.sin(np.deg2rad(theta)), np.cos(np.deg2rad(theta)), 0],[0, 0, 1]]
            #r = np.round(r,5)
            print(r)
    def ikine(self,px, py, pz, roll, pitch, yaw):
        mth = np.dot(self.rotarZ('deg',yaw), self.rotarY('deg',pitch), self.rotarX('deg',roll))
        print(mth)
        
        
        
        
if __name__ == "__main__":
    robot = Robot5R()
    print(robot)
    #robot.plot([0,0,0,0,0])
    #robot.plot([0,0,0,0,0], limits=[-25, 25, -25, 25, 0, 40])
    robot.teach([0,0,0,0,0])    #Instruccion para plotear robot con los sliders
    #robot.ikine(1,1,1,1,1,1)
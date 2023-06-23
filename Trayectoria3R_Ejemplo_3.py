import roboticstoolbox as rp
import roboticstoolbox as rtb
import numpy as np
from roboticstoolbox import *
import math
import matplotlib.pyplot as plt

class Robot3R(DHRobot):
    def __init__(self):
        deg = math.pi/180
        #Medidas de los eslabones en metros
        l1 = 0.065;
        l2 = 0.084;
        l3 = 0.19;
        #Crear links
        R = []
        R.append(RevoluteDH(d=l1, a=0, alpha=math.pi / 2))
        R.append(RevoluteDH(d=0, a=l2, alpha=0))
        R.append(RevoluteDH(d=0, a=l3, alpha=0))
        super().__init__(R, name="3R", keywords=("3r",))
        self.qz = np.zeros(3)
        self.qr = [0, 45*math.pi/180, 90*math.pi/180]
        self.addconfiguration("qz", self.qz)
        self.addconfiguration("qr", self.qr)
robot = Robot3R()
print(robot)
qt = rtb.tools.trajectory.jtraj(robot.qz, robot.qr,40)
# ax = plt.subplot(2,1,2)
# plt.show()
robot.plot(qt.q, limits=[-0.45, 0.45, -0.45, 0.45, 0, 0.40])
plt.clf()
#ax = robot.plot(qt.q, limits=[-0.45, 0.45, -0.45, 0.45, 0, 0.40], fig=figura) #     pass()
#q = [0, 45*math.pi/180, 90*math.pi/180]
# Plot the joint trajectory with a 50ms delay between configurations

#robot.plot(qt.q, limits=[-0.45, 0.45, -0.45, 0.45, 0, 0.40])
        
# puma = rtb.models.DH.Puma560()
# print(puma)
# #puma.plot(puma.qz)
# T = puma.fkine([0.1,0.2,0.3,0.4,0.5,0.6])
# print(T)
# puma.jacob0([0.1,0.2,0.3,0.4,0.5,0.6])
# #puma.plot(puma.qz)
# qt = rtb.tools.trajectory.jtraj(puma.qr, puma.qz,50)
# print(qt)
# puma.plot(qt.q)




        

# #!/usr/bin/env python
# """
# @author Jesse Haviland
# """
# 
# import roboticstoolbox as rp
# import numpy as np
# 
# # Make a panda robot
# panda = rp.models.DH.Panda()
# 
# # Init joint to the 'ready' joint angles
# panda.q = panda.qr
# 
# # Make 100 random sets of joint angles
# q = np.random.rand(, 7)
# 
# # Plot the joint trajectory with a 50ms delay between configurations
# panda.plot(q=q, backend='pyplot', dt=2, vellipse=False, fellipse=False)
# # panda.plot(q=q, backend='swift', dt=0.050, vellipse=False, fellipse=False)
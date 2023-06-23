import roboticstoolbox as rp
import numpy as np
from roboticstoolbox import *
import roboticstoolbox as rtb
import math
import numpy as np
import matplotlib.pyplot as plt


class Robot3R(DHRobot):        
    def IK(l1, l2, l3, Px, Py, Pz):  # Cinematica Inversa
        b = math.sqrt(Px ** 2 + Py ** 2)
        c = Pz - l1
        e = math.sqrt(b ** 2 + c ** 2)
        theta1 = math.atan2(Py, Px)
        cost3 = (e ** 2 - l2 ** 2 - l3 ** 2) / (2 * l2 * l3)
        sent3 = -math.sqrt(1 - (cost3) ** 2)
        theta3 = math.atan2(sent3, cost3)
        alpha = math.atan2(c, b)
        phi = math.atan2((l3 * sent3), (l2 + l3 * cost3))
        theta2 = alpha - phi
        theta1 = round(theta1, 5)
        theta2 = round(theta2, 5)
        theta3 = round(theta3, 5)
#         if (theta2 >= -46*math.pi/180 and theta2 <= math.pi):
#             theta2 = round(theta2 + math.pi / 4, 5)
#         if (theta3 >= -91*math.pi/180 and theta3 <= math.pi):
#             theta3 = round(theta3 + math.pi / 2, 5)
        #print('theta1: ' + str(round(math.degrees(theta1),5)) + ' ---- ' + str(theta1))
        #print('theta2: ' + str(round(math.degrees(theta2),5)) + ' ---- ' + str(theta2))
        #print('theta3: ' + str(round(math.degrees(theta3),5)) + ' ---- ' + str(theta3))
        return theta1, theta2, theta3
    def __init__(self):
        deg = math.pi/180
        #Medidas de los eslabones en metros
        l1 = 0.065;
        l2 = 0.084;
        l3 = 0.19;
        t1 = []
        t2 = []
        t3 = []
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

#Medidas de los eslabones
l1 = 6.5;
l2 = 8.4;
l3 = 19;
Px = [27.4, 23.251, 19, 0, 0, 0, 19, 26.737, 19, 0, -1, 0, 19, 22.141, 19, 0, -1.2, 0, 19, 26.061, 19, 0, 0, 0, 19, 27.4]
Py = [0, 0, 0, 19, 25.131, 19, 0, 0, 0, 19, 25.5, 19, 7, 7, 7, 19, 25.270, 19, 7, 6.983, 7, 19, 25, 19, 0, 0]
Pz = [6.5, 1.5, 14.9, 14.9, 0, 14.9, 14.9, 0.722, 14.9, 14.9, 3, 14.9, 14.9, 0, 14.9, 14.9, 7.5, 14.9, 14.9, 1.729, 14.9, 14.9, 8.5, 14.9, 14.9, 6.5]
t1 = np.zeros(26)
t2 = np.zeros(26)
t3 = np.zeros(26)
deg1 = []
deg2 = []
deg3 = []
aux = 0
#Punto1
for i in range(len(Px)):
    #print('***********PUNTO ' + str(i+1) + ' ************')
    [t1[i], t2[i], t3[i]] = Robot3R.IK(l1, l2, l3, Px[i], Py[i], Pz[i])
q = np.array([[t1[0], t2[0], t3[0]],[t1[1], t2[1], t3[1]]])
#print(q)
robot = Robot3R()


#Iniciar robot en qz [0,0,0]
#robot.plot(robot.qz)
for i in range(2,len(Px)):
    q = np.insert(q, i, [t1[i], t2[i], t3[i]],axis=0)
    #print(q)
def Trayectoria_ss(start,end,steps):
    qt = rtb.tools.trajectory.jtraj(start, end, steps)
    #print(np.round(np.rad2deg(qt.q)))
    for i in range(steps):
        deg1.append(np.round(np.rad2deg(qt.q[i,0]),2))
        deg2.append(np.round(np.rad2deg(qt.q[i,1] + 45*math.pi/180),2))
        deg3.append(np.round(np.rad2deg(qt.q[i,2] + 90*math.pi/180),2))
        print("ServoA :" + str(deg1[i]) + "    " + "ServoB :" + str(deg2[i]) + "    " + "ServoC :" + str(deg3[i]))
    robot.plot(qt.q, limits=[-0.40, 0.40, -0.40, 0.40, 0, 0.35])
    plt.clf()
    
def Trayectoria(start,end,steps):
    global aux
    for j in range(end-start):        
        qt = rtb.tools.trajectory.jtraj(q[start+j], q[start+j+1], steps)
        #print(np.round(np.rad2deg(qt.q)))
        robot.plot(qt.q, limits=[-0.40, 0.40, -0.40, 0.40, 0, 0.35])
        plt.clf()
        aux += 1
        for i in range(steps):
            if aux < 2:
                deg1.append(np.round(np.rad2deg(qt.q[i,0]),2))
                deg2.append(np.round(np.rad2deg(qt.q[i,1] + 45*math.pi/180),2))
                deg3.append(np.round(np.rad2deg(qt.q[i,2] + 90*math.pi/180),2))
                print("ServoA :" + str(deg1[i]) + "    " + "ServoB :" + str(deg2[i]) + "    " + "ServoC :" + str(deg3[i]))
            else:
                deg1.insert(i, np.round(np.rad2deg(qt.q[i,0]),2))
                deg2.insert(i, np.round(np.rad2deg(qt.q[i,1] + 45*math.pi/180),2))
                deg3.insert(i, np.round(np.rad2deg(qt.q[i,2] + 90*math.pi/180),2))
                print("ServoA :" + str(deg1[i]) + "    " + "ServoB :" + str(deg2[i]) + "    " + "ServoC :" + str(deg3[i]))
                deg1.pop(steps)
                deg2.pop(steps)
                deg3.pop(steps)
Trayectoria(2,5,5)






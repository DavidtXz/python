import roboticstoolbox as rp
import numpy as np
from roboticstoolbox import *
import roboticstoolbox as rtb
import math
import numpy as np
import matplotlib.pyplot as plt


class Robot3R(DHRobot):
    def __init__(self):
        deg = math.pi/180
        #Medidas de los eslabones en metros
        l1 = 6.5;
        l2 = 8.4;
        l3 = 19;
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
    def DK(l1, l2, l3, q0, q1, q2):  # Cinematica Directa
        MTH = robot.fkine([q0,q1,q2])
        #print(MTH)
        return MTH
        

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
deg4 = []
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
    qt = rtb.tools.trajectory.jtraj(q[start], q[end], steps)
    #print(np.round(np.rad2deg(qt.q)))
    for i in range(steps):
        deg1.append(np.round(np.rad2deg(qt.q[i,0]),2))
        deg2.append(np.round(np.rad2deg(qt.q[i,1] + 45*math.pi/180),2))
        deg3.append(np.round(np.rad2deg(qt.q[i,2] + 90*math.pi/180),2))
        print("ServoA :" + str(deg1[i]) + "    " + "ServoB :" + str(deg2[i]) + "    " + "ServoC :" + str(deg3[i]))
    robot.plot(qt.q, limits=[-0.40, 0.40, -0.40, 0.40, 0, 0.35])
    plt.clf()
    deg1.clear()
    deg2.clear()
    deg3.clear()
    
def Trayectoria2(start,end,steps):
    for j in range(end-start):        
        qt = rtb.tools.trajectory.jtraj(q[start+j], q[start+j+1], steps)
        #print(np.round(np.rad2deg(qt.q)))
        robot.plot(qt.q, limits=[-40, 40, -40, 40, 0, 35])
        plt.clf()
        for i in range(steps):
            deg1.append(np.round(np.rad2deg(qt.q[i,0]),2))
            deg2.append(np.round(np.rad2deg(qt.q[i,1] + 45*math.pi/180),2))
            deg3.append(np.round(np.rad2deg(qt.q[i,2] + 90*math.pi/180),2))
            print("ServoA :" + str(deg1[i]) + "    " + "ServoB :" + str(deg2[i]) + "    " + "ServoC :" + str(deg3[i]))
        deg1.clear()
        deg2.clear()
        deg3.clear()       
                
def Trayectoria(start,end,steps):
    sensor1 = 1
    sensor2 = 1
    auxsensor2 = 1
    sensor3 = 1
    vel = 2
    for j in range(end-start):
        print("***** Punto " + str(j) + " - " + str(j+1) + " *******")
        qt = rtb.tools.trajectory.jtraj(q[start+j], q[start+j+1], steps)
        #print(np.round(np.rad2deg(qt.q)))
        robot.plot(qt.q, limits=[-40, 40, -40, 40, 0, 35])
        plt.clf()
        for i in range(steps):
            if sensor1 == 0:
                while sensor1 == 0:
                    print("Robot Detenido")
                    # time.sleep(0.05)
                    if sensor1 == 1:
                        print("Trayectoria Normal")
                        break
#             if i == 5:
#                 sensor2 = 0
            if sensor2 == 0:              
                
                print("Velocidad reducida")
                if i == 0 and len(deg1) == 0:
                    qt = rtb.tools.trajectory.jtraj(q[start+j], q[start+j+1], steps*vel)
                elif i == 0 and len(deg1) !=0:
                    i = 1
                    q0 = np.deg2rad(deg1[i-1])
                    q1 = np.deg2rad(deg2[i-1] - 45)
                    q2 = np.deg2rad(deg3[i-1] - 90)
                    qt = rtb.tools.trajectory.jtraj([q0, q1, q2], q[start+j+1], steps*vel)
                else:
                    q0 = np.deg2rad(deg1[i-1])
                    q1 = np.deg2rad(deg2[i-1] - 45)
                    q2 = np.deg2rad(deg3[i-1] - 90)
                    qt = rtb.tools.trajectory.jtraj([q0, q1, q2], q[start+j+1], steps*vel)

                deg1.clear()
                deg2.clear()
                deg3.clear()
                robot.plot(qt.q, limits=[-40, 40, -40, 40, 0, 35])
                plt.clf()
                for k in range(steps*vel):
#                     if k == 14:
#                         sensor2 = 1
                    if sensor2 == 1:
                        print("Trayectoria Normal")
                        if k == 0 and len(deg1) == 0:
                            qt = rtb.tools.trajectory.jtraj(q[start+j], q[start+j+1], int(steps/vel))
                        elif k == 0 and len(deg1) !=0:
                            k = 1
                            q0 = np.deg2rad(deg1[k-1])
                            q1 = np.deg2rad(deg2[k-1] - 45)
                            q2 = np.deg2rad(deg3[k-1] - 90)
                            qt = rtb.tools.trajectory.jtraj([q0, q1, q2], q[start+j+1], int(steps/vel))
                        else:
                            q0 = np.deg2rad(deg1[k-1])
                            q1 = np.deg2rad(deg2[k-1] - 45)
                            q2 = np.deg2rad(deg3[k-1] - 90)
                            qt = rtb.tools.trajectory.jtraj([q0, q1, q2], q[start+j+1], int(steps/vel))

                        deg1.clear()
                        deg2.clear()
                        deg3.clear()
                        robot.plot(qt.q, limits=[-40, 40, -40, 40, 0, 35])
                        plt.clf()
                        for h in range (int(steps/vel)):
                            
                            plt.clf()
                            deg1.append(np.round(np.rad2deg(qt.q[h,0]),2))
                            deg2.append(np.round(np.rad2deg(qt.q[h,1] + 45*math.pi/180),2))
                            deg3.append(np.round(np.rad2deg(qt.q[h,2] + 90*math.pi/180),2))
                            print("ServoA :" + str(deg1[h]) + "    " + "ServoB :" + str(deg2[h]) + "    " + "ServoC :" + str(deg3[h]))
                        break
                    #
                    #
                    #
                    deg1.append(np.round(np.rad2deg(qt.q[k,0]),2))
                    deg2.append(np.round(np.rad2deg(qt.q[k,1] + 45*math.pi/180),2))
                    deg3.append(np.round(np.rad2deg(qt.q[k,2] + 90*math.pi/180),2))
                    print("ServoA :" + str(deg1[k]) + "    " + "ServoB :" + str(deg2[k]) + "    " + "ServoC :" + str(deg3[k]))
                break                              
            deg1.append(np.round(np.rad2deg(qt.q[i,0]),2))
            deg2.append(np.round(np.rad2deg(qt.q[i,1] + 45*math.pi/180),2))
            deg3.append(np.round(np.rad2deg(qt.q[i,2] + 90*math.pi/180),2))
            print("ServoA :" + str(deg1[i]) + "    " + "ServoB :" + str(deg2[i]) + "    " + "ServoC :" + str(deg3[i]))
        deg1.clear()
        deg2.clear()
        deg3.clear()
        
        
def PickandPlace(gripper, steps):
    sensor1 = 1
    sensor2 = 1
    auxsensor2 = 1
    sensor3 = 1
    vel = 2
    start = 80 * (math.pi/180)
    end = 135 * (math.pi/180)
    if gripper == "open":
        print("Abriendo Gripper")
        qg = rtb.tools.trajectory.jtraj(start, end, steps)            
        for i in range(steps):
            if sensor1 == 0:
                while sensor1 == 0:
                    print("Robot Detenido")
                    # time.sleep(0.05)
                    if sensor1 == 1:
                        print("Abriendo Gripper")
                        break
#             if i == 5:
#                 sensor2 = 0
            if sensor2 == 0:              
                print("Velocidad reducida")
                if i == 0 and len(deg4) == 0:
                    qg = rtb.tools.trajectory.jtraj(start, end, steps*vel)
                elif i == 0 and len(deg4) !=0:
                    i = 1
                    q3 = np.deg2rad(deg4[i-1])
                    qg = rtb.tools.trajectory.jtraj(q3, end, steps*vel)
                else:
                    q3 = np.deg2rad(deg4[i-1])
                    qg = rtb.tools.trajectory.jtraj(q3, end, steps*vel)
                deg4.clear()
                for k in range(steps*vel):
#                     if k == 14:
#                         sensor2 = 1
                    if sensor2 == 1:
                        print("Abriendo Gripper")
                        if k == 0 and len(deg4) == 0:
                            qg = rtb.tools.trajectory.jtraj(start, end, int(steps/vel))
                        elif k == 0 and len(deg4) !=0:
                            k = 1
                            q3 = np.deg2rad(deg4[k-1])
                            qg = rtb.tools.trajectory.jtraj(q3, end, int(steps/vel))
                        else:
                            q3 = np.deg2rad(deg4[k-1])
                            qg = rtb.tools.trajectory.jtraj(q3, end, int(steps/vel))
                        deg4.clear()
                        plt.clf()
                        for h in range (int(steps/vel)):
                            deg4.append(np.round(np.rad2deg(qg.q[h]),2))
                            print("***** Grados " + str(deg4[h]) + " *******")
                        break
                    deg4.append(np.round(np.rad2deg(qg.q[k]),2))
                    print("***** Grados " + str(deg4[k]) + " *******")
                break                              
            deg4.append(np.round(np.rad2deg(qg.q[i]),2))
            print("***** Grados " + str(deg4[i]) + " *******")
        deg4.clear()            
    elif gripper == "close":
        print("Cerrando Gripper")
        qg = rtb.tools.trajectory.jtraj(end, start, steps)            
        for i in range(steps):
            if sensor1 == 0:
                while sensor1 == 0:
                    print("Robot Detenido")
                    # time.sleep(0.05)
                    if sensor1 == 1:
                        print("Cerrando Gripper")
                        break
#             if i == 5:
#                 sensor2 = 0
            if sensor2 == 0:              
                print("Velocidad reducida")
                if i == 0 and len(deg4) == 0:
                    qg = rtb.tools.trajectory.jtraj(end, start, steps*vel)
                elif i == 0 and len(deg4) !=0:
                    i = 1
                    q3 = np.deg2rad(deg4[i-1])
                    qg = rtb.tools.trajectory.jtraj(q3, start, steps*vel)
                else:
                    q3 = np.deg2rad(deg4[i-1])
                    qg = rtb.tools.trajectory.jtraj(q3, start, steps*vel)
                deg4.clear()
                for k in range(steps*vel):
#                     if k == 14:
#                         sensor2 = 1
                    if sensor2 == 1:
                        print("Cerrando Gripper")
                        if k == 0 and len(deg4) == 0:
                            qg = rtb.tools.trajectory.jtraj(end, start, int(steps/vel))
                        elif k == 0 and len(deg4) !=0:
                            k = 1
                            q3 = np.deg2rad(deg4[k-1])
                            qg = rtb.tools.trajectory.jtraj(q3, start, int(steps/vel))
                        else:
                            q3 = np.deg2rad(deg4[k-1])
                            qg = rtb.tools.trajectory.jtraj(q3, start, int(steps/vel))
                        deg4.clear()
                        plt.clf()
                        for h in range (int(steps/vel)):
                            deg4.append(np.round(np.rad2deg(qg.q[h]),2))
                            print("***** Grados " + str(deg4[h]) + " *******")
                        break
                    deg4.append(np.round(np.rad2deg(qg.q[k]),2))
                    print("***** Grados " + str(deg4[k]) + " *******")
                break                              
            deg4.append(np.round(np.rad2deg(qg.q[i]),2))
            print("***** Grados " + str(deg4[i]) + " *******")
        deg4.clear()

#PickandPlace("open",20)
#PickandPlace("close",20)      
#Trayectoria(2,4,10)
#Trayectoria_vel(0,2,10)
Trayectoria2(0,2,20)





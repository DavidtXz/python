from roboticstoolbox import *
import roboticstoolbox as rtb
import math
import numpy as np
import matplotlib.pyplot as plt


import time


Velocidad = 1
    
Px = [27.4, 23.251, 19, 0, 0, 0, 19, 26.737, 19, 0, -1, 0, 19, 22.141, 19, 0, -1.2, 0, 19, 26.061, 19, 0, 0, 0, 19, 27.4]
Py = [0, 0, 0, 19, 25.131, 19, 0, 0, 0, 19, 25.5, 19, 7, 7, 7, 19, 25.270, 19, 7, 6.983, 7, 19, 25, 19, 0, 0]
Pz = [6.5, 1.5, 14.9, 14.9, 0, 14.9, 14.9, 0.722, 14.9, 14.9, 3, 14.9, 14.9, 0, 14.9, 14.9, 7.5, 14.9, 14.9, 1.729, 14.9, 14.9, 8.5, 14.9, 14.9, 6.5]
t1 = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
t2 = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
t3 = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
def PC_DK(q1, q2, q3, l1, l2, l3):  # Cinematica Directa
    R = []
    R.append(RevoluteDH(d=l1, a=0, alpha=math.pi / 2))
    R.append(RevoluteDH(d=0, a=l2, alpha=0))
    R.append(RevoluteDH(d=0, a=l3, alpha=0))
    robot = DHRobot(R, name='vector')
#     robot.plot([q1, q2, q3], limits=[-45, 45, -45, 45, 0, 40])  # Instruccion para plotear robot con los sliders para mover cada articulacion en la simulación.
    MTH = robot.fkine([q1, q2, q3])
    plt.clf()
    # print(MTH)
    
    return MTH


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
#     theta3 = round(theta3, 5)
#     if (theta2 >= -46*math.pi/180 and theta2 <= math.pi):
#         theta2 = round(theta2 + math.pi / 4, 5)
#     if (theta3 >= -91*math.pi/180 and theta3 <= math.pi):
#         theta3 = round(theta3 + math.pi / 2, 5)
    print('theta1: ' + str(round(math.degrees(theta1),5)) + ' ---- ' + str(theta1))
    print('theta2: ' + str(round(math.degrees(theta2),5)) + ' ---- ' + str(theta2))
    print('theta3: ' + str(round(math.degrees(theta3),5)) + ' ---- ' + str(theta3))
    return theta1, theta2, theta3
#Medidas de los eslabones
l1 = 6.5;
l2 = 8.4;
l3 = 19;
#Punto1
for j in range(len(Px)):
    print('***********PUNTO ' + str(j+1) + ' ************')
    [t1[j], t2[j], t3[j]] = IK(l1, l2, l3, Px[j], Py[j], Pz[j])


def PAtoPB (PuntoI, PuntoF, NumDatos):
    theta1_p1top2 = np.linspace(t1[PuntoI-1], t1[PuntoF-1], NumDatos)
    theta2_p1top2 = np.linspace(t2[PuntoI-1], t2[PuntoF-1], NumDatos)
    theta3_p1top2 = np.linspace(t3[PuntoI-1], t3[PuntoF-1], NumDatos)
    return theta1_p1top2, theta2_p1top2, theta3_p1top2


def Trayectoria(PuntoI, PuntoF, NumDatos):
    global Velocidad
    deg1 = np.round(np.linspace(t1[PuntoI - 1], t1[PuntoF - 1], NumDatos), 2)
    deg2 = np.round(np.linspace(t2[PuntoI - 1], t2[PuntoF - 1], NumDatos), 2)
    deg3 = np.round(np.linspace(t3[PuntoI - 1], t3[PuntoF - 1], NumDatos), 2)
    theta1_p1top2 = np.linspace(t1[PuntoI-1], t1[PuntoF-1], NumDatos)
    theta2_p1top2 = np.linspace(round(t2[PuntoI-1] + 45*math.pi/180, 5), round(t2[PuntoF-1] + 45*math.pi/180, 5), NumDatos)
    theta3_p1top2 = np.linspace(round(t3[PuntoI-1] + 90*math.pi/180, 5), round(t3[PuntoF-1] + 90*math.pi/180, 5), NumDatos)
    for k in range(NumDatos):
#         pca.servo[0].angle = deg1[k]
#         time.sleep(0.01)
#         pca.servo[1].angle = deg2[k]
#         time.sleep(0.01)
#         pca.servo[2].angle = 180 - deg3[k]
#         time.sleep(0.01)

        print('************** PASO ' + str(k+1) + ' ****************' )
        print('SERVOMOTOR_A = ' + str(deg1[k]) + ' SERVOMOTOR_B = ' + str(deg2[k]) + ' SERVOMOTOR_C = ' + str(deg3[k]))
        print('Eslabon_A = ' + str(theta1_p1top2[k]) + ' Eslabon_B = ' + str(theta2_p1top2[k]) + ' Eslabon_C = ' + str(theta3_p1top2[k]))
        mth = PC_DK(deg1[k], deg2[k], deg3[k], l1, l2, l3)

            
def Pick_Place(PuntoI, PuntoF, NumDatos):
    pp = np.round(np.linspace(PuntoI, PuntoF, NumDatos),2)
    for k in range(NumDatos):
#         pca.servo[3].angle = pp[k]
        time.sleep(0.01)
        print('GRIPPER ' + str(pp[k]))

Trayectoria(1,2,int(10 * Velocidad))
print(t1)
# Pick_Place(135,80,int(250 * Velocidad))#Pick
# Trayectoria(2,3,int(10 * Velocidad))
# Trayectoria(3,4,int(10 * Velocidad))
# Trayectoria(4,5,int(10 * Velocidad))    
# Pick_Place(80,135,int(250 * Velocidad))#Place
# Trayectoria(5,6,int(10 * Velocidad))
# Trayectoria(6,7,int(10 * Velocidad))
# Trayectoria(7,8,int(10 * Velocidad))
# Pick_Place(135,80,int(250 * Velocidad))#Pick
# Trayectoria(8,9,int(10 * Velocidad))
# Trayectoria(9,10,int(10 * Velocidad))
# Trayectoria(10,11,int(10 * Velocidad))
# Pick_Place(80,135,int(250 * Velocidad))#Place
# Trayectoria(11,12,int(10 * Velocidad))
# Trayectoria(12,13,int(10 * Velocidad))
# Trayectoria(13,14,int(10 * Velocidad))
# Pick_Place(135,80,int(250 * Velocidad))#Pick
# Trayectoria(14,15,int(10 * Velocidad))
# Trayectoria(15,16,int(10 * Velocidad))
# Trayectoria(16,17,int(10 * Velocidad))
# Pick_Place(80,135,int(250 * Velocidad))#Place
# Trayectoria(17,18,int(10 * Velocidad))
# Trayectoria(18,19,int(10 * Velocidad))
# Trayectoria(19,20,int(10 * Velocidad))
# Pick_Place(135,80,int(250 * Velocidad))#Pick
# Trayectoria(20,21,int(10 * Velocidad))
# Trayectoria(21,22,int(10 * Velocidad))
# Trayectoria(22,23,int(10 * Velocidad))
# Pick_Place(80,135,int(250 * Velocidad))#Place
# Trayectoria(23,24,int(10 * Velocidad))
# Trayectoria(24,25,int(10 * Velocidad))
# Trayectoria(25,26,int(10 * Velocidad))


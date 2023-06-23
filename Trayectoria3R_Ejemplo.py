from roboticstoolbox import *
import matplotlib.pyplot as plt
import roboticstoolbox as rtb
import math
import numpy as np
import time
import numpy
from spatialmath.base import *
from spatialmath import *
import matplotlib.pyplot as plt
#import matplotlib as plt
mx = []
my = []
mz = []
def PC_DK(q1,q2,q3,l1,l2,l3):
    R = []
    R.append(RevoluteDH(d=l1, a=0, alpha=math.pi / 2))
    R.append(RevoluteDH(d=0, a=l2, alpha=0))
    R.append(RevoluteDH(d=0, a=l3, alpha=0))
    robot = DHRobot(R, name='vector')
    robot.plot([q1, q2, q3], name='vector', fig = fig, limits=[-25,25,-25,25,0,40])  # Instruccion para plotear robot con los sliders para mover cada articulacion en la simulación.
    
    #id = env.add(robot)
    MTH = robot.fkine([q1,q2,q3])
    print(MTH)
    return MTH
l1 = 6.5
l2 = 8.4
l3 = 19
q3 = 0
#Interpolacion de angulos
theta1_P1toP2 = np.linspace(0, math.pi/2, 10)
theta2_P1toP2 = np.linspace(0, math.pi/2, 10)
theta3_P1toP2 = np.linspace(0, 0, 10)
fig = plt.figure(figsize=(4, 4))
for i in range(0,10,1):
    mth = PC_DK(theta1_P1toP2[i],theta2_P1toP2[i],theta3_P1toP2[i],l1,l2,l3)
#     print('mth.t[1]=' + str(mth.t[0]))
#     print('mth.t[2]=' + str(mth.t[1]))
#     print('mth.t[3]=' + str(mth.t[2]))
#     print(type(mth.t[0]))
#    time.sleep(0.0)
    mx.append(mth.t[0])
    my.append(mth.t[1])
    mz.append(mth.t[2])
    print(mth)
# plt.plot(mx, my, mz, 'x', color = 'blue')
# plt.show()

    #print()
    #print(type(mth))
    #print(len(mth))
'''print(robot) #Instruccion para imprimir convencion DH.
robot.plot([math.pi/2, math.pi/2, math.pi/2]) #Instruccion para plotear el robot espeficicando los
                                              #angulos de posicion inicial para cada articulación
robot.teach([math.pi*(0/180), math.pi*(0/180), math.pi*(0/180)])    #Instruccion para plotear robot con los sliders
                                                        #para mover cada articulacion en la simulación.
print(robot.fkine([math.pi*(110/180), math.pi*(150/180), math.pi*(90/180)]))  #Instruccion para imprimir la matriz de
                                                            #transformacion homogenea (MTH)'''

'''for j in range(-30,135,10):
    q2 = math.radians(j)
    for i in range(0,180,10):
        if (i%2) == 0:
            q1 = math.radians(i)
            robot.plot(q1,q2,q3) #Instruccion para plotear robot con los sliders para mover cada articulacion en la simulación.
            print(robot.fkine([q1,q2,q3])) #Instruccion para imprimir la matriz de transformacion homogenea (MTH)
        else:
            q1 = math.radians(180-i)
            robot.plot(q1,q2,q3)  # Instruccion para plotear robot con los sliders para mover cada articulacion en la simulación.
            print(robot.fkine([q1,q2,q3]))  # Instruccion para imprimir la matriz de transformacion homogenea (MTH)
    i = 0'''

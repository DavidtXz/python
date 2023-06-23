from roboticstoolbox import *
import matplotlib.pyplot as plt
import roboticstoolbox as rtb
import math
from spatialmath.base import *
from spatialmath import *
import matplotlib as plt
R = []

R.append(RevoluteDH(d=0, a=10, alpha=0))
R.append(RevoluteDH(d=0, a=10, alpha=0))

robot = DHRobot(R, name='vector')
print(robot) #Instruccion para imprimir convencion DH.
robot.plot([math.pi/2, math.pi/2], limits=[-25, 25, -25, 25, 0, 40]) #Instruccion para plotear el robot espeficicando los
                                    #angulos de posicion inicial para cada articulación
robot.teach([math.pi*(110/180), math.pi*(150/180)])    #Instruccion para plotear robot con los sliders
                                                        #para mover cada articulacion en la simulación.
print(robot.fkine([math.pi*(110/180), math.pi*(150/180)]))  #Instruccion para imprimir la matriz de
                                                            #transformacion homogenea


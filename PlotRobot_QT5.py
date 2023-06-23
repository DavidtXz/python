import roboticstoolbox as rp
import numpy as np
from roboticstoolbox import *
import roboticstoolbox as rtb
import math
import numpy as np
import matplotlib.pyplot as plt
import time
from chau2 import *
#########Librerias para la interfaz###########
import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import*
from PyQt5.QtCore import QPropertyAnimation,QEasingCurve, Qt, QRegExp
from PyQt5.QtGui import QColor, QRegExpValidator, QFont
from PyQt5.uic import loadUi
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import cv2
from matplotlib.backends.backend_qt5agg import FigureCanvas

class Canvas_grafica(FigureCanvas):
    def __init__(self, parent = None):
        self.fig , self.ax = plt.subplots(1, dpi=100, figsize=(2, 2), sharey=True, facecolor='white')
        super().__init__(self.fig)
#         l1 = 6.5
#         l2 = 8.4
#         l3 = 19
#         
#         R = []
#         R.append(RevoluteDH(d=l1, a=0, alpha=math.pi / 2))
#         R.append(RevoluteDH(d=0, a=l2, alpha=0))
#         R.append(RevoluteDH(d=0, a=l3, alpha=0))
#         robot = DHRobot(R, name='vector')
#         robot.plot([0, 0, 0], limits=[-45, 45, -45, 45, 0,40])
        

class VentanaMain(QMainWindow):
    def __init__(self):
        super(VentanaMain, self).__init__()
        loadUi('Plot_PlotRobot.ui', self)
        self.grafica = Canvas_grafica()
        self.verticalLayout_2.addWidget(self.grafica)
        robot.plot([0, 0, 0], fig = self.grafica.fig, limits=[-45, 45, -45, 45, 0,40])
        
        
        
        
        
        
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = VentanaMain()
    window.show()
    sys.exit(app.exec_())
    #app.exec_()
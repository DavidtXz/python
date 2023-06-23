import sys
from screen import *

from PyQt5 import QtWidgets, QtCore

import matplotlib

import numpy as np

matplotlib.use('QT5Agg')



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

class DialogApplication(QtWidgets.QDialog):
    """
    """
    def __init__(self):
        
        
        
        
        super().__init__()

        # Set up the dialog
        self.dialog = Ui_Dialog()
        self.dialog.setupUi(self)
    
        self.axes = self.dialog.mplwidget.canvas.axes

        self.axes.plot(np.array([1,2,4,8,3,5,2]), color='tab:red')
        self.axesLeft = self.axes.twinx()
        self.axesLeft.plot(np.array([3,5,7,6,3,2,7]), color='tab:blue')
        l1 = 6.5
        l2 = 8.4
        l3 = 19
        R = []
        R.append(RevoluteDH(d=l1, a=0, alpha=math.pi / 2))
        R.append(RevoluteDH(d=0, a=l2, alpha=0))
        R.append(RevoluteDH(d=0, a=l3, alpha=0))
        robot = DHRobot(R, name='vector')
        robot.plot([0, 0, 0], vellipse=False, limits=[-45, 45, -45, 45, 0,40])
        
        
        

        # Show
        self.show()
        
if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    dialog = DialogApplication()
    dialog.show()
    sys.exit(app.exec_())






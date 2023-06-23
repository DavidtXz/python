import roboticstoolbox as rp
import numpy as np
from roboticstoolbox import *
from chau2 import *
import roboticstoolbox as rtb
import math
import numpy as np
import matplotlib.pyplot as plt
import time
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
from FigurasGeometricas import getContours
from matplotlib.backends.backend_qt5agg import FigureCanvas

#########Librerias para la Rpi4###########
# from adafruit_servokit import ServoKit
# import RPi.GPIO as GPIO
# import time
# from board import SCL, SDA
# import busio
# from PIL import Image, ImageDraw, ImageFont
# import adafruit_ssd1306
# GPIO.setwarnings(False)

class VentanaMain(QMainWindow):
    def __init__(self):
        super(VentanaMain, self).__init__()
        loadUi('GuiRobot.ui', self)
        #Barra Menu
        self.pushButton_auto.clicked.connect(lambda: self.stackedWidget.setCurrentWidget(self.page_auto))
        self.pushButton_semi.clicked.connect(lambda: self.stackedWidget.setCurrentWidget(self.page_semi))
        self.pushButton_manual.clicked.connect(lambda: self.stackedWidget.setCurrentWidget(self.page_manual))
        #Configuración Modo Manual
        self.horizontalSliderdeg1.valueChanged.connect(self.updateSlider1)
        self.horizontalSliderdeg2.valueChanged.connect(self.updateSlider2)
        self.horizontalSliderdeg3.valueChanged.connect(self.updateSlider3)
        self.horizontalSliderGripper.valueChanged.connect(self.updateSliderGripper)
        self.pushButton_manual_Gopen.clicked.connect(self.Gopen)
        self.pushButton_manual_Gclose.clicked.connect(self.Gclose)
        #Configuracion Modo SemiAutomático
        self.lineEdit.setAlignment(Qt.AlignCenter)
        self.lineEdit.setClearButtonEnabled(True)
        self.lineEdit.setPlaceholderText("Ingresar Posicion en X")
        self.lineEdit.setValidator(QRegExpValidator(QRegExp("[0-9]+[.]+[0-9]+"))) #Ingresar solo numeros
        self.lineEdit_2.setAlignment(Qt.AlignCenter)
        self.lineEdit_2.setClearButtonEnabled(True)
        self.lineEdit_2.setPlaceholderText("Ingresar Posicion en Y")
        self.lineEdit_2.setValidator(QRegExpValidator(QRegExp("[0-9]+[.]+[0-9]+"))) #Ingresar solo numeros
        self.lineEdit_3.setAlignment(Qt.AlignCenter)
        self.lineEdit_3.setClearButtonEnabled(True)
        self.lineEdit_3.setPlaceholderText("Ingresar Posicion en Z")
        self.lineEdit_3.setValidator(QRegExpValidator(QRegExp("[0-9]+[.]+[0-9]+"))) #Ingresar solo numeros
        self.pushButton_semi_enviar.clicked.connect(self.obtenerGrados)
        #Configuracion Modo Automático
        self.textEdit.setReadOnly(True)
        self.pushButton_iniciarTra.clicked.connect(self.Traj)
        #Instanciar worker1
        self.Worker1 = Worker1()
        self.Worker1.ImageUpdate.connect(self.ImageUpdateSlot)
        self.Worker1.start()
        #Instanciar grafico plot robot
        self.grafica = Canvas_grafica()
        self.verticalLayout_19.addWidget(self.grafica)
        
    def updateSlider1(self,event):
        time.sleep(0.1)
        plt.clf()
        self.lcdNumberdeg1.display(event)
        deg1 = np.deg2rad(self.horizontalSliderdeg1.value())
        deg2 = np.deg2rad(self.horizontalSliderdeg2.value())
        deg3 = np.deg2rad(self.horizontalSliderdeg3.value())
        robot.plot([deg1, deg2, deg3], vellipse=False, limits=[-45, 45, -45, 45, 0, 40])
    def updateSlider2(self,event):
        time.sleep(0.1)
        plt.clf()
        self.lcdNumberdeg2.display(event)
        deg1 = np.deg2rad(self.horizontalSliderdeg1.value())
        deg2 = np.deg2rad(self.horizontalSliderdeg2.value())
        deg3 = np.deg2rad(self.horizontalSliderdeg3.value())
        robot.plot([deg1, deg2, deg3], vellipse=False, limits=[-45, 45, -45, 45, 0, 40])
       #plt.show()
    def updateSlider3(self,event):
        time.sleep(0.1)
        plt.clf()
        self.lcdNumberdeg3.display(event)
        deg1 = np.deg2rad(self.horizontalSliderdeg1.value())
        deg2 = np.deg2rad(self.horizontalSliderdeg2.value())
        deg3 = np.deg2rad(self.horizontalSliderdeg3.value())
        robot.plot([deg1, deg2, deg3], vellipse=False, limits=[-45, 45, -45, 45, 0, 40])
        #plt.show()
    def updateSliderGripper(self,event):
        time.sleep(0.1)
        self.lcdNumberGripper.display(event)
        deg4 = np.deg2rad(self.horizontalSliderGripper.value())
        print(deg4)
    def obtenerGrados(self):
        px = float(self.lineEdit.text())
        py = float(self.lineEdit_2.text())
        pz = float(self.lineEdit_3.text())
        b = math.sqrt(px ** 2 + py ** 2)
        c = pz - l1
        e = math.sqrt(b ** 2 + c ** 2)
        theta1 = math.atan2(py, px)
        cost3 = (e ** 2 - l2 ** 2 - l3 ** 2) / (2 * l2 * l3)
        sent3 = -math.sqrt(1 - (cost3) ** 2)
        theta3 = math.atan2(sent3, cost3)
        alpha = math.atan2(c, b)
        phi = math.atan2((l3 * sent3), (l2 + l3 * cost3))
        theta2 = alpha - phi
        theta1 = round(np.rad2deg(theta1),2)
        theta2 = round(np.rad2deg(theta2),2)
        theta3 = round(np.rad2deg(theta3),2)
        self.lcdNumber_degpx.display(theta1)
        self.lcdNumber_degpy.display(theta2)
        self.lcdNumber_degpz.display(theta3)
        time.sleep(0.05)
        plt.clf()
        robot.plot([theta1, theta2, theta3], vellipse=False, limits=[-45, 45, -45, 45, 0, 40])
    def Traj(self):
        PickandPlace("open",20)
        Trayectoria(0,1,20)
        PickandPlace("close",20)
        Trayectoria(1,4,20)
        PickandPlace("close",20)
        Trayectoria(4,7,20)
        PickandPlace("open",20)
        Trayectoria(7,10,20)
        PickandPlace("close",20)
        Trayectoria(10,13,20)
        PickandPlace("open",20)
        Trayectoria(13,16,20)
        PickandPlace("close",20)
        Trayectoria(16,19,20)
        PickandPlace("open",20)
        Trayectoria(19,22,20)
        PickandPlace("close",20)
        Trayectoria(22,25,20)
    def Gopen(self):
        PickandPlace("open",20)
    def Gclose(self):
        PickandPlace("close",20)
    def ImageUpdateSlot(self, Image):
        self.label_10.setPixmap(QPixmap.fromImage(Image))

    def CancelFeed(self):
        self.Worker1.stop()

class Canvas_grafica(FigureCanvas):
    def __init__(self, parent = None):
        self.fig , self.ax = plt.subplots(1, dpi=100, figsize=(5, 5), sharey=True, facecolor='white')
        super().__init__(self.fig)
        l1 = 6.5
        l2 = 8.4
        l3 = 19
        
        R = []
        R.append(RevoluteDH(d=l1, a=0, alpha=math.pi / 2))
        R.append(RevoluteDH(d=0, a=l2, alpha=0))
        R.append(RevoluteDH(d=0, a=l3, alpha=0))
        robot = DHRobot(R, name='vector')
        robot.plot([0, 0, 0], fig = self.fig, limits=[-45, 45, -45, 45, 0,40])
        
class Worker1(QThread):
    ImageUpdate = pyqtSignal(QImage)
    def run(self):
        self.ThreadActive = True
        cap = cv2.VideoCapture(0)
        while self.ThreadActive:            
            ret, frame = cap.read()
            if ret:
                Image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                #getContours(Image)
                FlippedImage = cv2.flip(Image, 1)
                ConvertToQtFormat = QImage(FlippedImage.data, FlippedImage.shape[1], FlippedImage.shape[0], QImage.Format_RGB888)
                Pic = ConvertToQtFormat.scaled(640, 480, Qt.KeepAspectRatio)
                self.ImageUpdate.emit(Pic)
    def stop(self):
        self.ThreadActive = False
        self.quit()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = VentanaMain()
    window.show()
    sys.exit(app.exec_())
    #app.exec_()

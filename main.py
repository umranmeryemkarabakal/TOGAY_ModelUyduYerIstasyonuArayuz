from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from vtk import vtkRenderer, vtkSTLReader, vtkPolyDataMapper, vtkActor, vtkTransform
from PyQt5.QtCore import QElapsedTimer,QIODevice, pyqtSignal, QTimer, Qt, QThread, QObject, pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout
from PyQt5.QtSerialPort import QSerialPortInfo, QSerialPort
from PyQt5 import QtCore, QtGui, QtWidgets, QtOpenGL
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5 import QtGui, QtCore, QtWidgets
from PyQt5.QtGui import QImage, QPixmap  
from PyQt5.QtWidgets import *
from PyQt5 import QtWidgets
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from datetime import datetime
import pyqtgraph as pg
from gui import *
from OpenGL import GL, GLU
from stl import mesh
import numpy as np
import threading, wave, pyaudio,pickle,struct
import serial, serial.tools.list_ports
from threading import Thread, Event
import imutils, socket
import folium
import time
import socket
import pickle
import struct
import cv2
import csv
import sys
import io
import base64
import queue
import os
import sys
import asyncio
import websockets
import vtk
import struct

PAKET_NUMARASI = 0
UYDU_STATÜSÜ = 0
HATA_KODU = ""
GÖNDERME_SAATİ = "00/00/0000-00:00:00"
BASINÇ1 = 0.0
BASINÇ2 = 0.0
YÜKSEKLİK1 = 0.0
YÜKSEKLİK2 = 0.0
İRTİFA_FARKI = 0.0
İNİŞ_HIZI = 0.0
SICAKLIK = 0.0
PİL_GERİLİMİ = 0.0
GPS1_LATITUDE = 0.0
GPS1_LONGITUDE = 0.0
GPS1_ALTITUDE = 0.0
PITCH = 0.0
ROLL = 0.0
YAW = 0.0
RHRH = "0000"
IoT_DATA = 0.0
TAKIM_NO = "000000"

TakımID = "270432"

veriPaketi = []
headerList = ["PAKET NUMARASI","UYDU STATÜSÜ","HATA KODU","GÖNDERME SAATİ","BASINÇ1","BASINÇ2","YÜKSEKLİK1","YÜKSEKLİK2","İRTİFA FARKI","İNİŞ HIZI","SICAKLIK","PİL GERİLİMİ","GPS1 LATITUDE","GPS1 LONGITUDE","GPS1 ALTITUDE","PITCH","ROLL","YAW","RHRH","IoT Data","TAKIM NO"]
videoPath = ""

kameraIP = ""

q = queue.Queue(maxsize=10)
BUFF_SIZE = 65536

def lerp(start, end, t):
    return start + t * (end - start)


class Window(QMainWindow):

    def __init__(self):
        super().__init__()

        self.unitUI = Ui_MainWindow()
        self.unitUI.setupUi(self)

        self.serial = Communication()

        self.comboBoxesAddItem()
        self.actionTrigger()
        self.initFonks()

        self.serial.data_received.connect(self.updateTable)
        self.serial.data_received.connect(self.updateCSVFile)

        self.init_frame()

        self.video_receiver = VideoThread('localhost', 8888)
        self.video_receiver.frame_received.connect(self.update_frame)
        self.video_receiver.start()


    def init_frame(self):
        self.videoLabel = QLabel()
        self.videoLabel.setAlignment(Qt.AlignCenter)
        self.videoLabel.setMaximumSize(1000, 300)
        self.unitUI.verticalLayout.addWidget(self.videoLabel)

    def update_frame(self, frame):

        height, width, channel = frame.shape
        bytes_per_line = 3 * width
        q_img = QImage(frame.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
        self.videoLabel.setPixmap(QPixmap.fromImage(q_img))

    def actionTrigger(self):
        self.unitUI.pushButtonConnect.clicked.connect(self.fonkConnect)
        self.unitUI.pushButtonDisconnect.clicked.connect(self.fonkDisconnect)
        self.unitUI.pushButton_9.clicked.connect(self.sendMekanikFiltreleme)
        self.unitUI.pushButton_10.clicked.connect(self.tasiyiciAyir)

    def initFonks(self):
        self.init3DVisualizerWidget()
        self.initGraphs()
        self.initMap()
        self.initTexts()
        self.initCSVFile()
        self.initTable()
        self.initTimers()

    def initTimers(self):
        self.timerUpdate3dVisualizer = QTimer()
        self.timerUpdate3dVisualizer.timeout.connect(self.update3DVisualizer)
        self.timerUpdateMap = QTimer()
        self.timerUpdateMap.timeout.connect(self.updateMap)
        self.timerUpdateText = QTimer()
        self.timerUpdateText.timeout.connect(self.updateTexts)

        self.timerUpdateGraph1 = QTimer()
        self.timerUpdateGraph1.timeout.connect(self.updateGraph1)
        self.timerUpdateGraph2 = QTimer()
        self.timerUpdateGraph2.timeout.connect(self.updateGraph2)
        self.timerUpdateGraph3 = QTimer()
        self.timerUpdateGraph3.timeout.connect(self.updateGraph3)
        self.timerUpdateGraph4 = QTimer()
        self.timerUpdateGraph4.timeout.connect(self.updateGraph4)
        self.timerUpdateGraph5 = QTimer()
        self.timerUpdateGraph5.timeout.connect(self.updateGraph5)
        #self.timerUpdateGraph6 = QTimer()
        #self.timerUpdateGraph6.timeout.connect(self.updateGraph6)

    def startTimers(self):
        self.timerUpdate3dVisualizer.start(1000)
        self.timerUpdateMap.start(1000)
        self.timerUpdateText.start(1000)

        self.timerUpdateGraph1.start(1000)
        self.timerUpdateGraph2.start(1000)
        self.timerUpdateGraph3.start(1000)
        self.timerUpdateGraph4.start(1000)
        self.timerUpdateGraph5.start(1000)
        #self.timerUpdateGraph6.start(1000)

    def stopTimers(self):
        self.timerUpdate3dVisualizer.stop()
        self.timerUpdateMap.stop()
        self.timerUpdateText.stop()

        self.timerUpdateGraph1.stop()
        self.timerUpdateGraph2.stop()
        self.timerUpdateGraph3.stop()
        self.timerUpdateGraph4.stop()
        self.timerUpdateGraph5.stop()
        #self.timerUpdateGraph6.stop()

    def initTexts(self):
        global TakımID,UYDU_STATÜSÜ, HATA_KODU
        global kameraIP

        GyroText = "x:\ty:\tz:\t" 
        self.unitUI.labelGyro.setText(GyroText)

        mapText = "GPS Latitude:\nGPS Longitude:\nGPS Altitude:" 
        self.unitUI.labelGPS.setText(mapText)           

        hataKoduText = "Hata Kodu:\n"
        self.unitUI.labelHataKodu.setText(hataKoduText)

        irtifaFarkıText = "İrtifa Farkı:\n"
        self.unitUI.labelIrtifaFarki.setText(irtifaFarkıText) 

        taşıyıcıBasınçText = "Taşıyıcı Basınç:\n"
        self.unitUI.labelTasiyicibasinc.setText(taşıyıcıBasınçText) 

        taşıyıcıYükseklikText = "Taşıyıcı Yükseklik:\n"
        self.unitUI.labelTasiyciYukseklik.setText(taşıyıcıYükseklikText)  

        takımIDText = f"Takım ID:\n{TakımID}"
        self.unitUI.labelTakmID.setText(takımIDText)
        
        uyduStatusuText = "Uydu statüsü:\n"
        self.unitUI.labelUyduStatusu.setText(uyduStatusuText)

        hataMesajiText = "Hata Mesajı:"
        self.unitUI.labelHataMesaji.setReadOnly(True)
        self.unitUI.labelHataMesaji.setAlignment(Qt.AlignCenter)
        self.unitUI.labelHataMesaji.setPlainText(hataMesajiText)

        self.hataMesajiList = []
        self.hataMesajiList.append(HATA_KODU)
    
    def updateTexts(self):
        global ROLL, PITCH, YAW, GPS1_LATITUDE, GPS1_LONGITUDE, GPS1_ALTITUDE, PAKET_NUMARASI, UYDU_STATÜSÜ, HATA_KODU, GÖNDERME_SAATİ, BASINÇ2, YÜKSEKLİK2
        global kameraIP

        GyroText = "x: {:.2f}\t".format(ROLL) + \
               "y: {:.2f}\t".format(PITCH) + \
               "z: {:.2f}".format(YAW)
        self.unitUI.labelGyro.setText(GyroText)

        mapText = "GPS Latitude: " + str(GPS1_LATITUDE) + "\n" + \
            "GPS Longitude: " + str(GPS1_LONGITUDE) + "\n" + \
            "GPS Altitude: " + str(GPS1_ALTITUDE) 
        self.unitUI.labelGPS.setText(mapText)
            
        hataKoduText = f"Hata Kodu:\n{HATA_KODU}"
        self.unitUI.labelHataKodu.setText(hataKoduText)

        irtifaFarkıText = f"İrtifa Farkı:\n{İRTİFA_FARKI}"
        self.unitUI.labelIrtifaFarki.setText(irtifaFarkıText) 

        taşıyıcıBasınçText = f"Taşıyıcı Basınç:\n{BASINÇ2}"
        self.unitUI.labelTasiyicibasinc.setText(taşıyıcıBasınçText) 

        taşıyıcıYükseklikText = f"Taşıyıcı Yükseklik:\n{YÜKSEKLİK2}"
        self.unitUI.labelTasiyciYukseklik.setText(taşıyıcıYükseklikText)  

        if UYDU_STATÜSÜ == 0 :
            uyduStatusuText = f"Uydu Statüsü:{UYDU_STATÜSÜ}\nUçuşa Hazır"
        elif UYDU_STATÜSÜ == 1 :
            uyduStatusuText = f"Uydu Statüsü:{UYDU_STATÜSÜ}\nYükselme"
        elif UYDU_STATÜSÜ == 2 :
            uyduStatusuText = f"Uydu Statüsü:{UYDU_STATÜSÜ}\nModel Uydu İniş"
        elif UYDU_STATÜSÜ == 3 :
            uyduStatusuText = f"Uydu Statüsü:{UYDU_STATÜSÜ}\nAyrılma"
        elif UYDU_STATÜSÜ == 4 :
            uyduStatusuText = f"Uydu Statüsü:{UYDU_STATÜSÜ}\nGörev Yükü İniş"
        elif UYDU_STATÜSÜ == 5 :
            uyduStatusuText = f"Uydu Statüsü:{UYDU_STATÜSÜ}\nKurtarma"
        self.unitUI.labelUyduStatusu.setText(uyduStatusuText)

        self.hataKoduLabels()

        if len(self.hataMesajiList) != 0:
            if HATA_KODU != self.hataMesajiList[-1]:
                self.hataMesajiLabels()
        self.hataMesajiList.append(HATA_KODU)

        if UYDU_STATÜSÜ == 3:
            self.tasiyiciAyirButton()

    def tasiyiciAyirButton(self):
        self.unitUI.pushButton_10.setText("Taşıyıcı\nAyrıldı")
        self.unitUI.pushButton_10.setEnabled(False)
        self.unitUI.pushButton_10.setStyleSheet("background-color: rgb(85, 145, 169);color:rgb(206, 215, 224);border-radius:5px;min-height: 30px;min-width: 80px;")
        self.unitUI.frame5_5.setStyleSheet("background-color: rgb(85, 145, 169);border-radius:5px;")

    def hataKoduLabels(self):
        global HATA_KODU

        labelList = [self.unitUI.label,self.unitUI.label_6,self.unitUI.label_5,self.unitUI.label_8,self.unitUI.label_10]
        for i in range(min(len(labelList), len(HATA_KODU))):
            if HATA_KODU[i] == "0":
                labelList[i].setStyleSheet("background-color: green;")
            elif HATA_KODU[i] == "1":
                labelList[i].setStyleSheet("background-color: red;")

    def hataMesajiLabels(self):
        global HATA_KODU

        hataMesaji = []

        if not (HATA_KODU.count("1")) == 0:
            if HATA_KODU[0] == "1":
                hataMesaji.append("model uydu iniş hızı sapma")
            if HATA_KODU[1] == "1":
                hataMesaji.append("görev yükü iniş hızı sapma")
            if HATA_KODU[2] == "1":
                hataMesaji.append("taşıyıcı basınç verisi alınamama")
            if HATA_KODU[3] == "1":
                hataMesaji.append("görev yükü konum verisi alınamama")
            if HATA_KODU[4] == "1":
                hataMesaji.append("ayrılma gerçekleşmeme")
        else:
            hataMesaji = ["problemsiz uçuş"]

        hataMesajiString = ', '.join(hataMesaji)
        hataMesajiString += " durumu"

        self.unitUI.labelHataMesaji.setPlainText(hataMesajiString)

    def comboBoxesAddItem(self):
        self.unitUI.comboBoxPort.clear()
        self.unitUI.comboBoxBaudRate.clear()

        serialPorts = serial.tools.list_ports.comports()
        serialPortNames = [port.device for port in serialPorts]
        self.unitUI.comboBoxPort.addItems(serialPortNames)

        baudRates = [9600,1200, 19200, 38400, 57600, 115200]
        self.unitUI.comboBoxBaudRate.addItems(map(str,baudRates))

    def fonkConnect(self):
        portName = self.unitUI.comboBoxPort.currentText()
        baudRate = int(self.unitUI.comboBoxBaudRate.currentText())

        self.serial.serialPort.port = portName
        self.serial.serialPort.baudrate = baudRate
        self.serial.connect()

        if self.serial.serialPort.is_open: #self.alive.isSet() and self.serialPort.is_open
            self.startTimers()

            self.unitUI.pushButtonConnect.setEnabled(False)
            self.unitUI.pushButtonDisconnect.setEnabled(True)

            self.unitUI.comboBoxPort.setEnabled(False)
            self.unitUI.comboBoxBaudRate.setEnabled(False)
    
    def fonkDisconnect(self):
        self.serial.disconnect()

        if not self.serial.serialPort.is_open:
            self.stopTimers()
            self.unitUI.pushButtonConnect.setEnabled(True)
            self.unitUI.pushButtonDisconnect.setEnabled(False)

            self.unitUI.comboBoxPort.setEnabled(True)
            self.unitUI.comboBoxBaudRate.setEnabled(True)

    def init3DVisualizerWidget(self):
        self.gyroWidget = QWidget()
        self.unitUI.verticalLayoutGyro.addWidget(self.gyroWidget)
        self.vbox = QVBoxLayout()        
        self.vtk_widget = QVTKRenderWindowInteractor(self.gyroWidget)
        self.vbox.addWidget(self.vtk_widget)
        self.gyroWidget.setLayout(self.vbox)

        self.yaw_window = []
        self.pitch_window = []
        self.roll_window = []

        self.max_window_size = 4  
        self.current_yaw = 0.0
        self.current_pitch = 0.0
        self.current_roll = 0.0

        self.t = 0.0

        self.renderer = vtk.vtkRenderer()
        self.render_window = self.vtk_widget.GetRenderWindow()
        self.render_window.AddRenderer(self.renderer)
        self.render_interactor = self.vtk_widget.GetRenderWindow().GetInteractor()
        self.render_interactor.SetInteractorStyle(None)
        
        stl_reader = vtk.vtkSTLReader()
        stl_reader.SetFileName("scaled_model.stl")  

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(stl_reader.GetOutputPort())
        
        self.actor = vtk.vtkActor()
        self.actor.SetMapper(mapper)

        self.actor.GetProperty().SetColor(0.0, 0.4, 0.5) 

        self.transform = vtk.vtkTransform()
        self.actor.SetUserTransform(self.transform)

        self.renderer.AddActor(self.actor)
        self.renderer.SetBackground(1.0, 1.0, 1.0)

        camera = self.renderer.GetActiveCamera()
        camera.SetPosition(0, 0, 100)  
        camera.SetFocalPoint(0, 0, 0)  
        camera.SetViewUp(0, 1, 0) 
        
        self.renderer.ResetCamera()

        self.vtk_widget.Initialize()
        self.vtk_widget.Start()
    
    def is_outlier(self, value, window):
        if len(window) < self.max_window_size:
            return False  

        mean = np.mean(window)
        std_dev = np.std(window)

        z_score = (value - mean) / std_dev if std_dev > 0 else 0
        return abs(z_score) > 2.0

    def update3DVisualizer(self):
        global PITCH, YAW, ROLL

        self.t += 0.001
        self.t = min(self.t, 1.0)

        if len(self.yaw_window) >= self.max_window_size:
            self.yaw_window.pop(0)
            self.pitch_window.pop(0)
            self.roll_window.pop(0)

        self.yaw_window.append(YAW)
        self.pitch_window.append(PITCH)
        self.roll_window.append(ROLL)

        if self.is_outlier(YAW, self.yaw_window) or self.is_outlier(PITCH, self.pitch_window) or self.is_outlier(ROLL, self.roll_window):
            return

        if self.yaw_window:
            self.current_yaw = self.yaw_window[-1]
            self.current_pitch = self.pitch_window[-1]
            self.current_roll = self.roll_window[-1]

        self.current_yaw = lerp(self.current_yaw, YAW, self.t)
        self.current_roll = lerp(self.current_roll, ROLL, self.t)
        self.current_pitch = lerp(self.current_pitch, PITCH, self.t)

        self.transform.Identity()  
        self.transform.RotateX(round(self.current_pitch,1))  
        self.transform.RotateY(round(self.current_yaw,1))   
        self.transform.RotateZ(round(self.current_roll,1))  

        self.render_window.Render()

    def closeEvent(self, event):
        if hasattr(self, 'render_window'):
            self.render_window.Finalize()  
        if hasattr(self, 'vtk_widget'):
            self.vtk_widget.GetRenderWindow().Finalize() 
        
        self.video_receiver.stop()
        
        event.accept()

    def initGraphs(self):
        self.graphWidget1 = pg.PlotWidget(axisItems={'bottom': DateAxisItem(orientation='bottom')})
        self.graphWidget2 = pg.PlotWidget(axisItems={'bottom': DateAxisItem(orientation='bottom')})
        self.graphWidget3 = pg.PlotWidget(axisItems={'bottom': DateAxisItem(orientation='bottom')})
        self.graphWidget4 = pg.PlotWidget(axisItems={'bottom': DateAxisItem(orientation='bottom')})
        self.graphWidget5 = pg.PlotWidget(axisItems={'bottom': DateAxisItem(orientation='bottom')})
        #self.graphWidget6 = pg.PlotWidget(axisItems={'bottom': DateAxisItem(orientation='bottom')})

        self.graphWidget1.getAxis('bottom').enableAutoSIPrefix(False)
        self.graphWidget2.getAxis('bottom').enableAutoSIPrefix(False)
        self.graphWidget3.getAxis('bottom').enableAutoSIPrefix(False)
        self.graphWidget4.getAxis('bottom').enableAutoSIPrefix(False)
        self.graphWidget5.getAxis('bottom').enableAutoSIPrefix(False)
        #self.graphWidget6.getAxis('bottom').enableAutoSIPrefix(False)

        self.unitUI.horizontalLayoutGraph.addWidget(self.graphWidget1)
        spacerItem2 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.unitUI.horizontalLayoutGraph.addItem(spacerItem2)
        self.unitUI.horizontalLayoutGraph.addWidget(self.graphWidget2)
        self.unitUI.horizontalLayoutGraph.addWidget(self.graphWidget3)
        spacerItem3 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.unitUI.horizontalLayoutGraph.addItem(spacerItem3)

        self.unitUI.horizontalLayoutGraph1.addWidget(self.graphWidget4)
        spacerItem3 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.unitUI.horizontalLayoutGraph1.addItem(spacerItem3)
        self.unitUI.horizontalLayoutGraph1.addWidget(self.graphWidget5)
        #self.unitUI.horizontalLayoutGraph1.addWidget(self.graphWidget6)
        spacerItem4 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.unitUI.horizontalLayoutGraph1.addItem(spacerItem4)

        self.graphWidget1.setBackground('w')
        self.graphWidget2.setBackground('w')
        self.graphWidget3.setBackground('w')
        self.graphWidget4.setBackground('w')
        self.graphWidget5.setBackground('w')
        #self.graphWidget6.setBackground('w')

        self.graphWidget1.showGrid(x=True, y=True)
        self.graphWidget2.showGrid(x=True, y=True)
        self.graphWidget3.showGrid(x=True, y=True)
        self.graphWidget4.showGrid(x=True, y=True)
        self.graphWidget5.showGrid(x=True, y=True)
        #self.graphWidget6.showGrid(x=True, y=True)


        self.graphWidget1.setTitle('<html><font size="3" color="#131313">Sıcaklık Grafiği</font></html>')
        self.graphWidget2.setTitle('<html><font size="3" color="#131313">Pil Gerilimi Grafiği</font></html>')
        self.graphWidget3.setTitle('<html><font size="3" color="#131313">İniş Hızı Grafiği</font></html>')
        self.graphWidget4.setTitle('<html><font size="3" color="#131313">Basınç Grafiği</font></html>')
        self.graphWidget5.setTitle('<html><font size="3" color="#131313">Yükseklik Grafiği</font></html>')
        #self.graphWidget6.setTitle('<html><font size="3" color="#131313">IoT Data Grafiği</font></html>')

        self.graphWidget4.setLabel('left', 'Basınç (Pa)')
        self.graphWidget4.setLabel('bottom', 'Zaman (dk:sn)')

        self.graphWidget1.setLabel('left', 'Sıcaklık (°C)')
        self.graphWidget1.setLabel('bottom', 'Zaman (dk:sn)')

        self.graphWidget3.setLabel('left', 'Hız (m/s)')
        self.graphWidget3.setLabel('bottom', 'Zaman (dk:sn)')

        self.graphWidget2.setLabel('left', 'Pil Gerilimi (V)')
        self.graphWidget2.setLabel('bottom', 'Zaman (dk:sn)')

        self.graphWidget5.setLabel('left', 'Yükseklik (m)')
        self.graphWidget5.setLabel('bottom', 'Zaman (dk:sn)')

        #self.graphWidget6.setLabel('left', '')
        #self.graphWidget6.setLabel('bottom', 'Zaman (dk:sn)')

        self.curve1 = self.graphWidget1.plot(pen={'color': '#054569', 'width': 2})
        self.curve2 = self.graphWidget2.plot(pen={'color': '#054569', 'width': 2})
        self.curve3 = self.graphWidget3.plot(pen={'color': '#054569', 'width': 2})
        self.curve4 = self.graphWidget4.plot(pen={'color': '#054569', 'width': 2})
        self.curve5 = self.graphWidget5.plot(pen={'color': '#054569', 'width': 2})
        #self.curve6 = self.graphWidget6.plot(pen={'color': '#054569', 'width': 2})

        self.x_values1 = []
        self.y_values1 = []
        self.x_values2 = []
        self.y_values2 = []
        self.x_values3 = []
        self.y_values3 = []
        self.x_values4 = []
        self.y_values4 = []
        self.x_values5 = []
        self.y_values5 = []
        #self.x_values6 = []
        #self.y_values6 = []

    def updateGraph1(self):
        global SICAKLIK,GÖNDERME_SAATİ
        
        dt = datetime.strptime(GÖNDERME_SAATİ,"%d/%m/%Y-%H:%M:%S")
        timestamps = dt.timestamp()
        
        self.x_values1.append(timestamps)
        self.y_values1.append(float(SICAKLIK))

        x_values1 = self.x_values1[-5:]
        y_values1 = self.y_values1[-5:]

        self.graphWidget1.setYRange(self.y_values1[-1] -2,self.y_values1[-1] +2)

        y_ticks = np.linspace(y_values1[-1] - 2, y_values1[-1] + 2, 5)
        y_ticks = [round(v, 2) for v in y_ticks]
        self.graphWidget1.getPlotItem().getAxis('left').setTicks([[(v, str(v)) for v in y_ticks]])
        
        self.curve1.setData(x_values1, y_values1)

    def updateGraph2(self):
        global PİL_GERİLİMİ,GÖNDERME_SAATİ
        
        dt = datetime.strptime(GÖNDERME_SAATİ,"%d/%m/%Y-%H:%M:%S")
        timestamps = dt.timestamp()
        
        self.x_values2.append(timestamps)
        self.y_values2.append(float(PİL_GERİLİMİ))

        x_values2 = self.x_values2[-5:]
        y_values2 = self.y_values2[-5:]

        self.graphWidget2.setYRange(self.y_values2[-1] -2,self.y_values2[-1] +2)

        y_ticks = np.linspace(y_values2[-1] - 2, y_values2[-1] + 2, 5)
        y_ticks = [round(v, 2) for v in y_ticks]
        self.graphWidget2.getPlotItem().getAxis('left').setTicks([[(v, str(v)) for v in y_ticks]])
        
        self.curve2.setData(x_values2, y_values2)

    def updateGraph3(self):
        global İNİŞ_HIZI,GÖNDERME_SAATİ

        dt = datetime.strptime(GÖNDERME_SAATİ,"%d/%m/%Y-%H:%M:%S")
        timestamps = dt.timestamp()
        
        self.x_values3.append(timestamps)
        self.y_values3.append(float(İNİŞ_HIZI))

        x_values3 = self.x_values3[-5:]
        y_values3 = self.y_values3[-5:]

        self.graphWidget3.setYRange(self.y_values3[-1] - 2, self.y_values3[-1] + 2)

        y_ticks = np.linspace(y_values3[-1] - 2, y_values3[-1] + 2, 5)
        y_ticks = [round(v, 2) for v in y_ticks]
        self.graphWidget3.getPlotItem().getAxis('left').setTicks([[(v, str(v)) for v in y_ticks]])

        self.curve3.setData(x_values3, y_values3)

    def updateGraph4(self):
        global BASINÇ1,GÖNDERME_SAATİ
        
        dt = datetime.strptime(GÖNDERME_SAATİ,"%d/%m/%Y-%H:%M:%S")
        timestamps = dt.timestamp()
        
        self.x_values4.append(timestamps)
        self.y_values4.append(float(PİL_GERİLİMİ))

        x_values4 = self.x_values4[-5:]
        y_values4 = self.y_values4[-5:]

        self.graphWidget1.setYRange(self.y_values1[-1] -5, self.y_values1[-1] + 5)

        y_ticks = np.linspace(y_values4[-1] - 5, y_values4[-1] + 5, 5)
        y_ticks = [round(v, 2) for v in y_ticks]
        self.graphWidget4.getPlotItem().getAxis('left').setTicks([[(v, str(v)) for v in y_ticks]])

        self.curve4.setData(x_values4, y_values4)

    def updateGraph5(self):
        global YÜKSEKLİK1,GÖNDERME_SAATİ
        
        dt = datetime.strptime(GÖNDERME_SAATİ,"%d/%m/%Y-%H:%M:%S")
        timestamps = dt.timestamp()
        
        self.x_values5.append(timestamps)
        self.y_values5.append(float(YÜKSEKLİK1))

        x_values5 = self.x_values5[-5:]
        y_values5 = self.y_values5[-5:]

        self.graphWidget5.setYRange(self.y_values5[-1] - 2, self.y_values5[-1] + 2)

        y_ticks = np.linspace(y_values5[-1] - 2, y_values5[-1] + 2, 5)
        y_ticks = [round(v, 2) for v in y_ticks]
        self.graphWidget5.getPlotItem().getAxis('left').setTicks([[(v, str(v)) for v in y_ticks]])

        self.curve5.setData(x_values5, y_values5)

    def updateGraph6(self):
        global IoT_DATA,GÖNDERME_SAATİ
        
        dt = datetime.strptime(GÖNDERME_SAATİ,"%d/%m/%Y-%H:%M:%S")
        timestamps = dt.timestamp()

        self.x_values6.append(timestamps)
        self.y_values6.append(float(IoT_DATA))

        x_values6 = self.x_values6[-5:]
        y_values6 = self.y_values6[-5:]

        self.graphWidget6.setYRange(self.y_values6[-1] - 2, self.y_values6[-1] + 2)

        y_ticks = np.linspace(y_values6[-1] - 2, y_values6[-1] + 2, 5)
        y_ticks = [round(v, 2) for v in y_ticks]
        self.graphWidget6.getPlotItem().getAxis('left').setTicks([[(v, str(v)) for v in y_ticks]])

        self.curve6.setData(x_values6, y_values6)

    def initMap(self):
        global GPS1_LATITUDE, GPS1_LONGITUDE

        coordinate = (GPS1_LATITUDE,GPS1_LONGITUDE)
        self.mapp = folium.Map(
            location=coordinate,
            zoom_start=15,
        )

        folium.Marker([GPS1_LATITUDE, GPS1_LONGITUDE], popup='gps').add_to(self.mapp)

        data = io.BytesIO()
        self.mapp.save(data, close_file=False)

        self.web_view = QWebEngineView()
        self.web_view.setHtml(data.getvalue().decode())
        self.unitUI.verticalLayoutGPS.addWidget(self.web_view)

    def updateMap(self):
        global GPS1_LATITUDE, GPS1_LONGITUDE

        self.coordinate = [GPS1_LATITUDE, GPS1_LONGITUDE]

        self.mapp = folium.Map(
            location=self.coordinate,
            zoom_start=15,
        )

        folium.Marker([GPS1_LATITUDE, GPS1_ALTITUDE], popup='gps').add_to(self.mapp)
        
        data = io.BytesIO()
        self.mapp.save(data, close_file=False)
        self.web_view.setHtml(data.getvalue().decode())

    def initCSVFile(self):
        global headerList

        self.csv_dosyasi = 'veri.csv'
        with open(self.csv_dosyasi, 'w', newline='', encoding='utf-8') as csv_file:
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(headerList)

    def updateCSVFile(self):
        global veriPaketi

        with open(self.csv_dosyasi, 'a', newline='') as csv_file:
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(veriPaketi)

    def initTable(self):
        global headerList 

        self.tableWidget = QTableWidget()
        self.tableWidget.setStyleSheet("background-color: #CED7E0;")
        self.tableWidget.verticalHeader().hide()
        self.values = 0
        self.tableWidget.setRowCount(self.values + 1)
        self.tableWidget.setColumnCount(21)
        self.tableWidget.setRowHeight(0,10)
      
        for index, header in enumerate(headerList):
            header_item = QTableWidgetItem(header)
            header_item.setForeground(QColor("#131313"))
            self.tableWidget.setHorizontalHeaderItem(index, header_item)
        
        self.unitUI.verticalLayoutTable.addWidget(self.tableWidget)

    def updateTable(self):
        global veriPaketi
        
        def floatToStr(number):
            return str(number)
        
        if (len(veriPaketi) == 21):

            self.tableWidget.selectRow(self.values)

            table_list = veriPaketi
            table_list = list(map(floatToStr, table_list))
            index = 0
            while index <= 20:
                self.tableWidget.setItem(self.values, index, QTableWidgetItem(table_list[index]))
                self.tableWidget.setRowHeight(index,10)
                index += 1
            self.values += 1
            self.tableWidget.setRowCount(self.values + 1)
            self.tableWidget.setEditTriggers(QAbstractItemView.NoEditTriggers)
        else:
            pass

    def sendMekanikFiltreleme(self):
        R = self.unitUI.lineEdit_17.text()
        H = self.unitUI.lineEdit_18.text()
        R1 = self.unitUI.lineEdit_19.text()
        H1 = self.unitUI.lineEdit_20.text()

        command = R  + H  + R1 + H1
        self.serial.serialPort.write(command)

    def tasiyiciAyir(self):
        global yerIstasyonuSocket

        command = "tasiyici ayir"
        self.serial.serialPort.write(command)
        

class DateAxisItem(pg.AxisItem):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def tickStrings(self, values, scale, spacing):

        x = [datetime.fromtimestamp(value).strftime("%d/%m/%Y-%H:%M:%S") for value in values]
        time_list = [datetime.split('-')[1] for datetime in x]

        new_time_list = [t.split(':')[1] + ':' + t.split(':')[2] for t in time_list]

        return new_time_list

class VideoThread(QThread):
    frame_received = pyqtSignal(np.ndarray)

    def __init__(self, host, port, output_file='output.mp4'):
        super().__init__()
        self.host = host
        self.port = port
        self.output_file = output_file
        self.running = True
        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # MP4 codec belirleme
        self.out = cv2.VideoWriter(self.output_file, self.fourcc, 20.0, (640, 480))  # Çıkış dosyası ayarları

    def run(self):
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((self.host, self.port))

        data = b""
        payload_size = struct.calcsize("Q")

        while self.running:
            while len(data) < payload_size:
                packet = client_socket.recv(4*1024)
                if not packet: break
                data += packet

            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack("Q", packed_msg_size)[0]

            while len(data) < msg_size:
                data += client_socket.recv(4*1024)

            frame_data = data[:msg_size]
            data = data[msg_size:]

            frame = pickle.loads(frame_data)
            self.frame_received.emit(frame)  # Signal ile frame gönder

            # Frame'i dosyaya yaz
            self.out.write(frame)

        client_socket.close()
        self.out.release()  # Video dosyasını kapat

    def stop(self):
        self.running = False
        self.wait()  # İş parçacığının tamamlanmasını bekle

class Communication(QObject):
    data_received = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.serialPort = serial.Serial()

        self.t = None
        self.alive = Event()

    def connect(self):
        try:
            if not self.serialPort.is_open:
                self.serialPort.open()
                print("Bağlantı açıldı")
                self.startThread()
            else:
                print("Bağlantı zaten açık")

        except Exception as e:
            print(f"Bağlantı açılamadı: {e}")

    def disconnect(self):
        self.stopThread()
        if self.serialPort.is_open:
            self.serialPort.close()
            print("Bağlantı kapatıldı")

    def readData(self):
        global veriPaketi
        global PAKET_NUMARASI, UYDU_STATÜSÜ, HATA_KODU, GÖNDERME_SAATİ, BASINÇ1, BASINÇ2
        global YÜKSEKLİK1, YÜKSEKLİK2, İRTİFA_FARKI, İNİŞ_HIZI, SICAKLIK, PİL_GERİLİMİ
        global GPS1_LATITUDE, GPS1_LONGITUDE, GPS1_ALTITUDE, PITCH, ROLL, YAW
        global RHRH, IoT_DATA, TAKIM_NO

        while(self.alive.isSet() and self.serialPort.is_open):
            data = self.serialPort.readline().decode("utf-8").strip()
            received_data = data.split()
            
            if (len(received_data) == 7):

            #if (len(received_data) == 21):
                """
                PAKET_NUMARASI = int(received_data[0])
                UYDU_STATÜSÜ = int(received_data[1])
                HATA_KODU = str(received_data[2])
                GÖNDERME_SAATİ = string(received_data[3])
                BASINÇ1 = float(received_data[4])
                BASINÇ2 = float(received_data[5])
                YÜKSEKLİK1 = float(received_data[6])
                YÜKSEKLİK2 = float(received_data[7])
                İRTİFA_FARKI = float(received_data[8])
                İNİŞ_HIZI = float(received_data[9])
                SICAKLIK = float(received_data[10])
                PİL_GERİLİMİ = float(received_data[11])
                GPS1_LATITUDE = float(received_data[12])
                GPS1_LONGITUDE = float(received_data[13])
                GPS1_ALTITUDE = float(received_data[14])
                PITCH = float(received_data[15])
                ROLL = float(received_data[16])
                YAW = float(received_data[17])
                RHRH = str(received_data[18])
                IoT_DATA = float(received_data[19])
                TAKIM_NO = str(received_data[20])
                """


                now = datetime.now()
                GÖNDERME_SAATİ = now.strftime("%d/%m/%Y-%H:%M:%S")

                UYDU_STATÜSÜ = 0
                HATA_KODU = "00000"
                BASINÇ2 = 0.0
                YÜKSEKLİK2 = 0.0
                İRTİFA_FARKI = 0.0
                İNİŞ_HIZI = 0.0
                PİL_GERİLİMİ = 0.0
                GPS1_LATITUDE = 0.0
                GPS1_LONGITUDE = 0.0
                GPS1_ALTITUDE = 0.0
                RHRH = "0000"
                IoT_DATA = 0.0
                TAKIM_NO = "000000"

                PAKET_NUMARASI = int(received_data[0])
                
                SICAKLIK = float(received_data[4])
                BASINÇ1 = float(received_data[5])
                YÜKSEKLİK1 = float(received_data[6])

                ROLL = float(received_data[1])
                PITCH = float(received_data[2])
                YAW = float(received_data[3])

                veriPaketi = [
                    PAKET_NUMARASI, UYDU_STATÜSÜ, HATA_KODU, 
                    GÖNDERME_SAATİ, BASINÇ1, BASINÇ2, 
                    YÜKSEKLİK1, YÜKSEKLİK2, İRTİFA_FARKI, 
                    İNİŞ_HIZI, SICAKLIK, PİL_GERİLİMİ, 
                    GPS1_LATITUDE, GPS1_LONGITUDE, GPS1_ALTITUDE, 
                    PITCH, ROLL,YAW, RHRH, IoT_DATA, TAKIM_NO]
                
                veriPaketiStr = ",".join(map(str, veriPaketi))
                veriPaketiStr = veriPaketiStr.strip()

                self.data_received.emit(veriPaketiStr)
                

    def startThread(self):
        if self.t is None or not self.t.is_alive():
            self.t = Thread(target=self.readData)
            self.t.setDaemon(True)
            self.alive.set()
            self.t.start()
            print("İş parçacığı başlatıldı")

    def stopThread(self):
        if self.t and self.t.is_alive():
            self.alive.clear() 
            self.t.join()
            print("İş parçacığı durduruldu")
            self.t = None

def main():
    import sys
    app = QtWidgets.QApplication(sys.argv)
    window = Window()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()

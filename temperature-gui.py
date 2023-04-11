from PyQt5.QtWidgets import *
from PyQt5.QtGui import QPixmap
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import serial
import time
import sys

class main(QWidget):
    def __init__(self):
        super().__init__()

        font_path = 'digital-7.ttf' # digital type font
        QFontDatabase.addApplicationFont(font_path)
        font1 = QFont("digital-7", 60, 500)

        self.label = QLabel(self) # monitoring label and configrations
        self.label.setFont(font1)
        self.label.setMinimumHeight(400)
        self.label.setMinimumWidth(400)
        self.label.setStyleSheet("color: white; background-color: #171717;")
        self.label.setAlignment(Qt.AlignCenter)
        self.label.move(700,350)
        self.label.setText("0 °C")
 
        self.labelTitle = QLabel(self) # title label and configrations
        self.pixmap = QPixmap('photos//title.jpg')
        self.labelTitle.setPixmap(self.pixmap)
        self.labelTitle.resize(self.pixmap.width(), self.pixmap.height())
        self.labelTitle.move(250, 40)

        self.labelEngine= QLabel(self) # engine label and configrations
        self.pixmap = QPixmap('photos//engine.jpg')
        self.labelEngine.setPixmap(self.pixmap)
        self.labelEngine.resize(self.pixmap.width(), self.pixmap.height())
        self.labelEngine.move(80, 320)

        self.labelEngineAlert= QLabel(self) # engine alert label and configrations
        self.pixmap = QPixmap('photos//engine_alert.JPG')
        self.labelEngineAlert.setPixmap(self.pixmap)
        self.labelEngineAlert.resize(self.pixmap.width(), self.pixmap.height())
        self.labelEngineAlert.move(710, 750)

        self.labelEngineState= QLabel(self) # engine state label and configrations
        self.pixmap = QPixmap('photos//engine_state.JPG')
        self.labelEngineState.setPixmap(self.pixmap)
        self.labelEngineState.resize(self.pixmap.width(), self.pixmap.height())
        self.labelEngineState.move(710, 650)

        self.labelRangeOut= QLabel(self) # engine temperature range out label and configrations
        self.pixmap = QPixmap('photos//range_out.JPG')
        self.labelRangeOut.setPixmap(self.pixmap)
        self.labelRangeOut.resize(self.pixmap.width(), self.pixmap.height())
        self.labelRangeOut.move(715, 850)

        self.labelTemperature= QLabel(self) # temperature label and configrations
        self.pixmap = QPixmap('photos//temperature.JPG')
        self.labelTemperature.setPixmap(self.pixmap)
        self.labelTemperature.resize(self.pixmap.width(), self.pixmap.height())
        self.labelTemperature.move(700, 350)

        self.labelStateBulb= QLabel(self) # engine state red alert lamb and configrations
        self.pixmap = QPixmap('photos//red.JPG')
        self.labelStateBulb.setPixmap(self.pixmap)
        self.labelStateBulb.resize(self.pixmap.width(), self.pixmap.height())
        self.labelStateBulb.move(650, 650)

        self.labelAlertBulb= QLabel(self) # engine state red alert lamb and configrations
        self.pixmap = QPixmap('photos//red.JPG')
        self.labelAlertBulb.setPixmap(self.pixmap)
        self.labelAlertBulb.resize(self.pixmap.width(), self.pixmap.height())
        self.labelAlertBulb.move(650, 750)

        self.labelRangeOutBulb= QLabel(self) # engine range out red alert lamb and and configrations
        self.pixmap = QPixmap('photos//red.JPG')
        self.labelRangeOutBulb.setPixmap(self.pixmap)
        self.labelRangeOutBulb.resize(self.pixmap.width(), self.pixmap.height())
        self.labelRangeOutBulb.move(650, 850)

        self.connectButton = QPushButton(self) # connect button and configrations
        self.connectButton.move(0, 650)
        self.connectButton.setFont(font1)
        self.connectButton.setIcon(QIcon("photos//connect.JPG"))
        self.connectButton.setIconSize(QSize(self.pixmap.width(), self.pixmap.height())*10)
        self.connectButton.setMaximumSize(600,300)
        self.connectButton.clicked.connect(self.openSerialPort)
        self.connectButton.setStyleSheet("QPushButton { border: none; }")

        self.setStyleSheet("background-color: #171717;")
        self.setWindowTitle(" ")
        self.showMaximized()
        self.setFixedSize(1400,1000)
        self.show()

    def openSerialPort(self):
        
        communication = serial.Serial(port = 'COM1', baudrate=9600, timeout=.1) # Select your com port number, baudrate and timeout

        while True:
            if communication:
                print("Serial communication succesful")
                while True: # get the data by usart and show the monitoring label
                    if communication.in_waiting > 0:
                        data = communication.readline() 
                        getData = data.decode().strip('\x00').split("|") 
                        temperature = getData[1] 
                        self.label.setText(temperature+" °C")
                        tempState = int(temperature)
                        tempState = 10

                        if tempState > 90: # determine bulbs state
                            self.labelStateBulb.clear()
                            self.pixmap = QPixmap('photos//green.JPG')
                            self.labelStateBulb.setPixmap(self.pixmap)
                            self.update()
                        else:
                            self.labelStateBulb.clear()
                            self.pixmap = QPixmap('photos//red.JPG')
                            self.labelStateBulb.setPixmap(self.pixmap)
                            self.update()

                        if tempState > 110:
                            self.labelAlertBulb.clear()
                            self.pixmap = QPixmap('photos//green.JPG')
                            self.labelAlertBulb.setPixmap(self.pixmap)
                            self.update()
                        else:
                            self.labelAlertBulb.clear()
                            self.pixmap = QPixmap('photos//red.JPG')
                            self.labelAlertBulb.setPixmap(self.pixmap)
                            self.update()

                        if tempState < (-40) or tempState > (150):
                            self.labelStateBulb.clear()
                            self.pixmap = QPixmap('photos//red.JPG')
                            self.labelStateBulb.setPixmap(self.pixmap)
                            self.labelRangeOutBulb.clear()
                            self.labelAlertBulb.clear()
                            self.pixmap = QPixmap('photos//red.JPG')
                            self.labelAlertBulb.setPixmap(self.pixmap)
                            self.pixmap = QPixmap('photos//green.JPG')
                            self.labelRangeOutBulb.setPixmap(self.pixmap)
                            self.update()
                        else:
                            self.labelRangeOutBulb.clear()
                            self.pixmap = QPixmap('photos//red.JPG')
                            self.labelRangeOutBulb.setPixmap(self.pixmap)
                            self.update()
                            
                        QApplication.processEvents()
            else:
                print("Serial communication not open")

app = QApplication([])
window = main()
app.exec()
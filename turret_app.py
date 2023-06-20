from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5.QtGui import QIcon, QPixmap, QImage
from PyQt5.uic import loadUi

from arduino_communication import ArduinoCommunication

import sys, time

class ConnectPopUp(QWidget):    # pop-up de conexão
    def __init__(self, parent):
        super().__init__() 
        self.parent = parent # guarda referência ao programa principal
        self.ui = loadUi('Interface/popup.ui', self) # carrega layout do pop-up

        self.COM_entry = self.ui.COM_entry # entrada de texto criada no arquivo popup.ui
        self.connect_button = self.ui.connect_button # botão de conexão criado no arquivo popup.ui
        self.connect_button.clicked.connect(self.connectToArduino) # conecta o botão de conexão ao método connectToArduino
        self.parent.popup = True # indica que o pop-up está aberto

    def connectToArduino(self):
        port = self.COM_entry.text() # lê a caixa de texto para obter a porta COM
        if (self.parent.communication.connect(port)): # tenta conectar ao Arduino
            self.close() # fecha o pop-up, retorna ao programa principal
            self.parent.uiConnected()
        else:
            self.COM_entry.setText("Conexão falhou!") # se não conseguir conectar, mostra mensagem de erro

    def closeEvent(self, event): # quando o pop-up é fechado
        self.parent.popup = False # indica que o pop-up foi fechado

class NerfApp(QWidget):
    def __init__(self):
        super().__init__()
        self.ui = loadUi('Interface/gui.ui', self)
        self.show()

        self.COM_port = ""
        self.popup = False
        self.connected = False
        self.motor_on = False
        self.laser_on = False
        self.shoot = False
        self.on_frame = False

        self.x = 0.5
        self.y = 0.5

        self.frame = self.ui.frame
        self.bluetooth_button = self.ui.bluetooth_button
        self.motor_on_button = self.ui.motor_on_button
        self.laser_on_button = self.ui.laser_on_button

        self.communication = ArduinoCommunication(self)
        #self.dist = 0

        self.bluetooth_button.clicked.connect(self.connectPopUp)
        self.motor_on_button.clicked.connect(self.motorOnOff)
        self.laser_on_button.clicked.connect(self.laserOnOff)

    def connectPopUp(self):
        if not self.connected and not self.popup:
            popup = ConnectPopUp(self)
            popup.show()

    def uiConnected(self):
        self.motor_on_button.setEnabled(True)
        self.laser_on_button.setEnabled(True)
        self.frame.setEnabled(True)
        bt_icon = QIcon('Interface/bt_connected.png')
        self.bluetooth_button.setIcon(bt_icon)

    def motorOnOff(self):
        if self.connected:
            self.motor_on = self.motor_on_button.isChecked()
            if self.motor_on:
                self.motor_on = 1
            else:
                self.motor_on = 0
            self.sendToArduino()

    def laserOnOff(self):
        if self.connected:
            self.laser_on = self.laser_on_button.isChecked()
            if self.laser_on:
                self.laser_on = 1
            else:
                self.laser_on = 0
            self.sendToArduino()

    def sendToArduino(self):
        if self.connected:
            message = bytes([255, int(self.x), int(self.y), int(self.motor_on), int(self.shoot), int(self.laser_on), 254])
            #message = bytes([255, 0, 0, 0, 0, self.laser_on, 254])

            self.communication.send_message(message)
            

    def remap(self, value, new_range_min, new_range_max, old_range_min, old_range_max): # remapeia valores de 70 a 550 para 0 a 253 

        remapped_val = (value - old_range_min) * (new_range_max - new_range_min) / (
                    old_range_max - old_range_min) + new_range_min

        if (remapped_val > new_range_max):
            remapped_val = new_range_max
        elif (remapped_val < new_range_min):
            remapped_val = new_range_min

        return remapped_val

    def mouseMoveEvent(self, event):
        if (69 < event.x() < 551 and 69 < event.y() < 551):   # se o mouse estiver dentro do frame
            self.x = int(self.remap(event.x(), 0, 253, 70, 550))
            self.y = int(self.remap(event.y(), 0, 253, 70, 550))
            self.on_frame = True
        else:
            self.on_frame = False
            self.shoot = False

        self.sendToArduino()

    def mousePressEvent(self, event):
        if self.on_frame and self.motor_on:
            self.shoot = 1
            self.sendToArduino()

    def mouseReleaseEvent(self, event):
        if self.on_frame:
            self.shoot = 0
            self.sendToArduino()

    

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = NerfApp()
    sys.exit(app.exec_())
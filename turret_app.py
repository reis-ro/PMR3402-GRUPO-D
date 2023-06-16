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

        self.x = 0.5
        self.y = 0.5

        self.frame = self.ui.frame
        self.bluetooth_button = self.ui.bluetooth_button
        self.motor_on_button = self.ui.motor_on_button
        #self.laser_on_button = self.ui.laser_on_button

        self.communication = ArduinoCommunication(self)

        self.bluetooth_button.clicked.connect(self.connectPopUp)
        self.motor_on_button.clicked.connect(self.motorOnOff)
        #self.laser_on_button.clicked.connect(self.laserOnOff)

    def connectPopUp(self):
        if not self.connected and not self.popup:
            popup = ConnectPopUp(self)
            popup.show()

    def uiConnected(self):
        self.motor_on_button.setEnabled(True)
        self.frame.setEnabled(True)
        new_button_img = QIcon('Interface/bt_connected.png')
        self.bluetooth_button.setIcon(new_button_img)

    def motorOnOff(self):
        if self.connected:
            self.motor_on = self.motor_on_button.isChecked()
            if self.motor_on:
                self.motor_on = 'L'
            else:
                self.motor_on = 'D'
            self.sendToArduino()

    # def laserOnOff(self):
    #     if self.connected:
    #         self.laser_on = self.laser_on_button.isChecked()
    #         self.sendToArduino()

    def sendToArduino(self):
        if self.connected:
            #message = [255, self.x, self.y, self.motor_on, self.shoot, 254]
            message = [self.motor_on]
            
            for i in message:
                self.communication.send_message(str(i).encode())
                time.sleep(2)
            

    def handle_data_received(self, data):
        print("Dados recebidos:", data)

    def closeEvent(self, event):
        self.communication.close()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = NerfApp()
    sys.exit(app.exec_())
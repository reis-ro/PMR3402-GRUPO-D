from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5.QtGui import QIcon, QPixmap, QImage
from PyQt5.uic import loadUi

import arduino_communication

import sys

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
            self.parent.set_ui()
        else:
            self.COM_entry.setText("Conexão falhou!") # se não conseguir conectar, mostra mensagem de erro

    def closeEvent(self, event): # quando o pop-up é fechado
        self.parent.popup = False # indica que o pop-up foi fechado

class NerfApp(QWidget):    # janela principal
    def __init__(self):
        super().__init__()

        self.ui = loadUi('Interface/gui.ui', self)  # carrega layout da interface
        self.show()     

        self.COM_port = ""         # Porta de comunicação com Arduino
        self.popup = False
        self.connected = False     # True quando conectado ao arduino
        self.motor_on = False      # True para ligar o motor
        self.laser_on = False      # True para ligar o laser
        self.shoot = False         # True para atirar

        self.frame = self.ui.frame                  # frame criado no arquivo gui.ui
        self.bluetooth_button = self.ui.bluetooth_button    # botão bluetooth criado no arquivo gui.ui
        self.motor_on_button = self.ui.motor_on_button      # botão motor criado no arquivo gui.ui

        self.communication = arduino_communication.ArduinoCommunication(self)  # objeto de comunicação com Arduino

        self.bluetooth_button.clicked.connect(self.connectPopUp)   # conecta o botão bluetooth ao método connectPopUp (abre pop-up de conexão)

    def connectPopUp(self):    # abre pop-up de conexão
        if not self.connected and not self.popup:
            popup = ConnectPopUp(self)
            popup.show()

    # def motor_on_off(self):    # liga/desliga motor
    # adicionar código para alterar estado do botão e do motor

    # def laser_on_off(self):    # liga/desliga laser
    # adicionar código para alterar estado do botão e do laser


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = NerfApp()
    app.exec_()
    #sys.exit(app.exec_())
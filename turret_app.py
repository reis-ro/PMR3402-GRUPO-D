from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5.QtGui import QIcon, QPixmap, QImage
from PyQt5.uic import loadUi

import sys

class NerfApp(QWidget):    # janela principal

    def __init__(self):
        super().__init__()

        self.ui = loadUi('Interface/gui.ui', self)  # carrega layout da interface
        self.show()     

        self.connected = False     # True quando conectado ao arduino
        self.motor_on = False      # True para ligar o motor
        self.laser_on = False      # True para ligar o laser
        self.shoot = False         # True para atirar

        self.on_pad = False        # True quando o cursor está sobre o pad
        self.pad_label = self.ui.pad_label                  # pad criado no arquivo gui.ui
        self.bluetooth_button = self.ui.bluetooth_button    # botão bluetooth criado no arquivo gui.ui
        self.motor_on_button = self.ui.motor_on_button      # botão motor criado no arquivo gui.ui

        # ADICIONAR COMUNICAÇÃO COM ARDUINO

    # def motor_on_off(self):    # liga/desliga motor
    # adicionar código para alterar estado do botão e do motor

    # def laser_on_off(self):    # liga/desliga laser
    # adicionar código para alterar estado do botão e do laser


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = NerfApp()
    app.exec_()
    #sys.exit(app.exec_())
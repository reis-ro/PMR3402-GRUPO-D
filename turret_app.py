import sys
from PyQt5.QtCore import QIODevice, QThread, pyqtSignal
from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5.QtGui import QIcon, QPixmap, QImage
from PyQt5.uic import loadUi
from PyQt5.QtSerialPort import QSerialPort
import json

class SerialThread(QThread):
    data_received = pyqtSignal(bytes)

    def __init__(self, port, parent=None):
        super().__init__(parent)
        self.port = port

    def run(self):
        self.serial_port = QSerialPort(self.port)
        self.serial_port.setBaudRate(QSerialPort.Baud9600)
        self.serial_port.readyRead.connect(self.handle_ready_read)

        if self.serial_port.open(QIODevice.ReadWrite):
            print("Conexão estabelecida com sucesso.")

        self.exec_()

    def handle_ready_read(self):
        data = self.serial_port.readAll()
        self.data_received.emit(data)

    def send_data(self, data):
        if self.serial_port.isOpen():
            self.serial_port.write(data)

    def close(self):
        if self.serial_port.isOpen():
            self.serial_port.close()

class ArduinoCommunication():
    def __init__(self, app):
        self.parent = app
        self.baudrate = 9600

    def connect(self, port):
        try:
            self.serial_thread = SerialThread(port)
            self.serial_thread.data_received.connect(self.parent.handle_data_received)  # Conexão assíncrona entre threads
            self.serial_thread.start()
            self.parent.connected = True
            self.parent.COM_port = port
            print("Arduino conectado na porta", port)
            return True

        except Exception as e:
            print("A conexão falhou:", str(e))
            return False

    def send_message(self, message):
        self.serial_thread.send_data(message)
        print(message)

    def close(self):
        self.serial_thread.close()

class ConnectPopUp(QWidget):
    def __init__(self, parent):
        super().__init__()
        self.parent = parent
        self.ui = loadUi('Interface/popup.ui', self)

        self.COM_entry = self.ui.COM_entry
        self.connect_button = self.ui.connect_button
        self.connect_button.clicked.connect(self.connectToArduino)
        self.parent.popup = True

    def connectToArduino(self):
        port = self.COM_entry.text()
        if self.parent.communication.connect(port):
            self.close()
            self.parent.uiConnected()
        else:
            self.COM_entry.setText("Conexão falhou!")

    def closeEvent(self, event):
        self.parent.popup = False

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
        self.laser_on_button = self.ui.laser_on_button

        self.communication = ArduinoCommunication(self)

        self.bluetooth_button.clicked.connect(self.connectPopUp)
        self.motor_on_button.clicked.connect(self.motorOnOff)
        self.laser_on_button.clicked.connect(self.laserOnOff)

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
            self.sendToArduino()

    def laserOnOff(self):
        if self.connected:
            self.laser_on = self.laser_on_button.isChecked()
            self.sendToArduino()

    def sendToArduino(self):
        if self.connected:
            #message = {'start': 255, 'x':self.x, 'y':self.y, 'motor':self.motor_on, 'shoot':self.shoot, 'end':254}
            message = {'value':1}
            json_data = json.dumps(message).encode()
            self.communication.send_message(json_data)

    def handle_data_received(self, data):
        print("Dados recebidos:", data)

    def closeEvent(self, event):
        self.communication.close()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = NerfApp()
    sys.exit(app.exec_())

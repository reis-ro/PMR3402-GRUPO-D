import serial

class ArduinoCommunication():
    def __init__(self, app):
        self.parent = app
        self.baudrate = 9600

    def connect(self, port):
        try:
            self.ser = serial.Serial(port, self.baudrate)
            self.parent.connected = True
            print("Arduino conectado na porta ", port)
            return True
        
        except:
            print("A conexao falhou!")
            return False

    def send_message(self, message):
        self.ser.write(message)
        print(message)
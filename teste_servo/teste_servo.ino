#include <Servo.h>

Servo servo;
Servo vertical;
int comando;

void setup() {
  Serial1.begin(9600); // Inicializa a porta serial do Arduino Mega (Serial0)
  servo.attach(2); // Conecta o servo ao pino 4
  // vertical.attach(3);
  // vertical.write(90);
}

void loop() {
  if (Serial1.available()) {
    comando = Serial1.parseInt(); // Lê o comando enviado via Bluetooth

    if (comando != 0){
      Serial1.println(comando);
      servo.write(comando);
      delay(100);
    }
    
  }
  
}

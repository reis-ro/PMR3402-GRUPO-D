#define MOTOR1_PIN1 9
#define MOTOR1_PIN2 10
#define MOTOR2_PIN1 11
#define MOTOR2_PIN2 12

int comando;

void desligaLaser() {
  digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, LOW);
  digitalWrite(MOTOR2_PIN1, LOW);
  digitalWrite(MOTOR2_PIN2, LOW);
}

void ligaLaser() {
  digitalWrite(MOTOR1_PIN1, HIGH);
  digitalWrite(MOTOR1_PIN2, LOW);
  digitalWrite(MOTOR2_PIN1, LOW);
  digitalWrite(MOTOR2_PIN2, LOW);
}

void setup(){

// Define pino dos motores DC
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
  pinMode(MOTOR2_PIN1, OUTPUT);
  pinMode(MOTOR2_PIN2, OUTPUT);

  desligaLaser();

  // Comunicação serial
  Serial.begin(9600);
  Serial1.begin(9600);// Conecta o servo ao pino 4

}

void loop(){
  if (Serial1.available() > 0) {
    comando = Serial1.read(); // Lê o comando enviado via Bluetooth
    if (comando == 'L') {
      ligaLaser();
    } else if (comando == 'D') {
      desligaLaser();
    }
  }
  
}


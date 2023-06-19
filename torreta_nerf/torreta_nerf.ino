#include <SoftwareSerial.h>
#include <Servo.h>

// Definição dos pinos utilizados pelos componentes
#define SERVO_HORIZONTAL_PIN 3
#define SERVO_VERTICAL_PIN 5
#define LASER_PIN 7
#define MOTOR1_PIN1 12
#define MOTOR1_PIN2 13
#define MOTOR2_PIN1 14
#define MOTOR2_PIN2 15
#define SERVO_EMPURRAR_PIN 11
// módulo HC05
#define RX 2
#define TX 3
#define BT_PIN 4

// Variáveis e definições associadas aos dados de comunicação serial
byte byte_from_app;

// Estado do propulsor
bool PROPULSION = OFF;

// Estado do laser
bool LASER = OFF;

// Definição dos estados da máquina de estados
typedef enum {
  STATE_IDLE,
  STATE_BT_CON,
  STATE_LASER_ON,
  STATE_DC_ON,
  STATE_LASER_DC_ON
} State;

// Inicialização dos objetos dos servos
Servo servoHorizontal;
Servo servoVertical;
Servo servoEmpurrar;

// Conecta o pino TX do HC05 ao pino RX 2 do arduino e pino RX do HC05 ao pino TX 3 do arduino por um divisor de tensão
SoftwareSerial BTserial(RX, TX);

// Inicializa variável de controle da máquina de estados
State currentState = STATE_IDLE;

void setup() {
  // Anexa servos aos pinos
  servoHorizontal.attach(SERVO_HORIZONTAL_PIN);
  servoVertical.attach(SERVO_VERTICAL_PIN);
  servoEmpurrar.attach(SERVO_EMPURRAR_PIN);

  // Inicialização da posição dos servos
  servoEmpurrar.write(RECOIL_REST);
  servoHorizontal.write(90);
  delay(1000);
  servoVertical.write(105);

  // Define pino do laser
  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);

  // Define pino dos motores DC
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
  pinMode(MOTOR2_PIN1, OUTPUT);
  pinMode(MOTOR2_PIN2, OUTPUT);

  // Define pino do bluetooth
  pinMode(BT_PIN, INPUT); 
  
  // Comunicação serial
  Serial.begin(9600);
}

void loop() {

  getData(); // Recebe os dados
  
  // Máquina de estados
  switch (currentState) {
    case STATE_IDLE:
      // Ação: Mantém a torreta parada em espera e reseta tudo
          // Transição para o próximo estado
      if (bluetoothConectado()) {
        currentState = STATE_BT_CON;
      }
      break;
      
    case STATE_BT_CON:
      // Ação: Mantém a torreta parada com BT conectado
      LASER = OFF;       // Desliga laser  
      PROPULSION = OFF;  // Desliga propulsores

      // Verificar o comando recebido e realizar ação correspondente
      if () { // botão do laser ativado
        // Executar ação correspondente ao comando 'laser'
        LASER = ON;       // Liga laser
      } 
      else if () { // botão de propulsão ativado
        // Executar ação correspondente ao comando 'motor_dc'
        PROPULSION = ON;  // Liga propulsores
      } 

      // Transições para outros estados
      if (!bluetoothConectado()) { // Verificação da conexão Bluetooth
        currentState = STATE_IDLE;
      } 
      if (laserLigado() && !propulsionLigada()) {
        currentState = STATE_LASER_ON;
      } 
      if (!laserLigado() && propulsionLigada()) {
        currentState = STATE_DC_ON;
      } 
      break;
      
    case STATE_LASER_ON:
      // Ação: Ativa o laser
      LASER = ON;       // Liga laser  
      PROPULSION = OFF;  // Desliga propulsores

      // Verificar o comando recebido e realizar ação correspondente
      if () { // botão do laser desativado
        // Executar ação correspondente ao comando 'laser'
        LASER = OFF;       // Desliga laser
      } 
      if () { // botão de propulsão ativado
        // Executar ação correspondente ao comando 'motor_dc'
        PROPULSION = ON;  // Liga propulsores
      } 
      
      // Transições para outros estados
      if (!bluetoothConectado()) {
        currentState = STATE_IDLE;
      } 
      if (!laserLigado()) {
        currentState = STATE_BT_CON;
      } 
      if (propulsionLigada()) {
        currentState = STATE_LASER_DC_ON;
      } 
      break;
      
    case STATE_DC_ON:
      // Ação: Ativa a propulsão dos dardos
      LASER = OFF;       // Desliga laser  
      PROPULSION = ON;  // Liga propulsores

      // Verificar o comando recebido e realizar ação correspondente
      if () { // botão do laser ativado
        // Executar ação correspondente ao comando 'laser'
        LASER = ON;       // Liga laser
      } 
      if () { // botão de propulsão desativado
        // Executar ação correspondente ao comando 'motor_dc'
        PROPULSION = OFF;  // Desliga propulsores
      } 
      if () { // botão de disparo pressionado

      }
      
      // Transições para outros estados
      if (!bluetoothConectado()) {
        currentState = STATE_IDLE;
      } 
      if (!propulsionLigada()) {
        currentState = STATE_BT_CON;
      } 
      if (laserLigado()) {
        currentState = STATE_LASER_DC_ON;
      }
      break;
      
    case STATE_LASER_DC_ON:
      // Ação: Ativa o laser e a propulsão dos dardos
      LASER = ON;       // Liga laser  
      PROPULSION = ON;  // Liga propulsores
      
      // Verificar o comando recebido e realizar ação correspondente
      if () { // botão do laser desativado
        // Executar ação correspondente ao comando 'laser'
        LASER = OFF;       // Desliga laser
      } 
      if () { // botão de propulsão desativado
        // Executar ação correspondente ao comando 'motor_dc'
        PROPULSION = OFF;  // Desliga propulsores
      } 
      if () { // botão de disparo pressionado
        // Executar ação correspondente ao comando 'shoot'

      }

      // Transições para outros estados
      if (!bluetoothConectado()) {
        currentState = STATE_IDLE;
      }
      else if (!laserLigado()) {
        currentState = STATE_DC_ON;
      } 
      else if (!propulsionLigada()) {
        currentState = STATE_LASER_ON;
      }
      break;
  }
}

void getData() {
  if (Serial.available()) {  // Se houver dados disponíveis no serial

    byte_from_app = Serial.read(); 
}
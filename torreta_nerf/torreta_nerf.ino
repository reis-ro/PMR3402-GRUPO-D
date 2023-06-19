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

// Definições de movimento
#define PAN_LIMIT_1 0
#define PAN_LIMIT_2 180
#define TILT_LIMIT_1 65
#define TILT_LIMIT_2 180
#define RECOIL_REST 180  // Ângulo do servo na posição de descanso
#define RECOIL_PUSHED 125 // Ângulo do servo necessário para empurrar o dardo

// Variáveis e definições associadas aos dados de comunicação serial
#define buffSize 30
#define startMarker 255
#define endMarker 254
byte byte_from_app;
byte inputBuffer[buffSize];
byte bytesRecvd = 0;
boolean data_received = FALSE;

// Variáveis associadas aos módulo de propulsão e tempo de disparo
bool is_firing = FALSE;
bool can_fire = FALSE;
bool recoiling = FALSE;

unsigned long firing_start_time = 0;
unsigned long firing_current_time = 0;
const long firing_time = 150;

unsigned long recoil_start_time = 0;
unsigned long recoil_current_time = 0;
const long recoil_time = 2 * firing_time;

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
 
// Inicializa a variável de verificação da conexão bluetooth
boolean BTconnected = FALSE;

// Inicializa variável de controle da máquina de estados
State currentState = STATE_IDLE;

// Variáveis temporais para verificação 
unsigned long tempoAtual;     // Variável para armazenar o tempo atual
unsigned long ultimoEnvio;    // Variável para armazenar o tempo do último envio
const unsigned long intervaloEnvio = 1000; // Intervalo de 1s para envio

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

  // Inicializações de tempo
  tempoAtual = millis();     // Inicializa a variável tempoAtual
  ultimoEnvio = tempoAtual;  // Inicializa a variável ultimoEnvio (bluetooth)
}

void loop() {

  getData(); // Recebe o buffer de dados
  if (data_received) { 

  }
  
  // Máquina de estados
  switch (currentState) {
    case STATE_IDLE:
      // Ação: Mantém a torreta parada em espera e reseta tudo
      LASER = OFF;       // Desliga laser  
      PROPULSION = OFF;  // Desliga propulsores
      servoHorizontal.write(90);  // Reseta posição inicial
      servoVertical.write(105);    // Reseta posição inicial
      
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
      if (inputBuffer[4] == TRUE) { // botão do laser ativado
        // Executar ação correspondente ao comando 'laser'
        LASER = ON;       // Liga laser
      } 
      else if (inputBuffer[2] == TRUE) { // botão de propulsão ativado
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
      if (inputBuffer[4] == FALSE) { // botão do laser desativado
        // Executar ação correspondente ao comando 'laser'
        LASER = OFF;       // Desliga laser
      } 
      if (inputBuffer[2] == TRUE) { // botão de propulsão ativado
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
      if (inputBuffer[4] == TRUE) { // botão do laser ativado
        // Executar ação correspondente ao comando 'laser'
        LASER = ON;       // Liga laser
      } 
      if (inputBuffer[2] == FALSE) { // botão de propulsão desativado
        // Executar ação correspondente ao comando 'motor_dc'
        PROPULSION = OFF;  // Desliga propulsores
      } 
      if (inputBuffer[3] == TRUE) { // botão de disparo pressionado

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
      if (inputBuffer[4] == FALSE) { // botão do laser desativado
        // Executar ação correspondente ao comando 'laser'
        LASER = OFF;       // Desliga laser
      } 
      if (inputBuffer[2] == FALSE) { // botão de propulsão desativado
        // Executar ação correspondente ao comando 'motor_dc'
        PROPULSION = OFF;  // Desliga propulsores
      } 
      if (inputBuffer[3] == TRUE) { // botão de disparo pressionado
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
  
  // Atualiza o estado da máquina de estados com base nos comandos lidos
}

void getData() {
  //expected structure of data [start byte, pan amount, tilt amount, motor on, firing button pressed, laser on, end byte]
  //start byte = 255
  //pan amount = byte between 0 and 253
  //tilt amount = byte between 0 and 253
  //motor on = 0 for off, 1 on
  //laser on = 0 for off, 1 on
  //firing button pressed = 0 for not pressed, 1 for pressed
  //end byte = 254

  if (Serial.available()) {  // Se houver dados disponíveis no serial

    byte_from_app = Serial.read();   // Lê o próximo caractere disponível

    if (byte_from_app == startMarker) {     // Procura pelo byte inicial (255)
      bytesRecvd = 0;                   // reseta os bytes recebidos para 0 (para começar a armazenar no inputBuffer)
      data_received = false;
    }

    else if (byte_from_app == endMarker) {    // Procura pelo byte final (254)
      data_received = true;                // coloca data_received em true para o dado ser utilizado
    }

    else {                            // adiciona os bytes recebidos para o inputBuffer
      inputBuffer[bytesRecvd] = byte_from_app;     // adiciona caractere para o input buffer
      bytesRecvd++;                                // incrementa os bytes recebidos (atua como índice)
      if (bytesRecvd == buffSize) {    // apenas por segurança no caso do inputBuffer lotar (não deve acontecer)
        bytesRecvd = buffSize - 1;    // se bytesReceived é maior que o tamanho do buffer, então ajusta bytesReceived para ser menor
      }
    }
  }
}

// Lógica para verificar se o Bluetooth está conectado
bool bluetoothConectado() {
  BT_state_previous = BTconnected;
  if (digitalRead(BT_PIN) == HIGH) { // se bluetooth estiver conectado
    BTconnected = TRUE;
    if (BT_state_previous == FALSE) { // inicializa BTserial se ele já não estava ligado antes
      BTserial.begin(9600); 
    }
  }
  else {
    BTconnected = FALSE;
  }
  return BTconnected;
}

// Lógica para verificar se o laser está ligado
bool laserLigado() {
  if (LASER) {
    return ON;
  }
  else {
    return OFF;
  }
}

// Lógica para verificar se a propulsão está ligada
bool propulsionLigada() {
  if (PROPULSION) {
    return ON;
  }
  else {
    return OFF;
  }
}

// Liga o laser
void ligarLaser() {
  digitalWrite(LASER_PIN, HIGH);
}

// Liga a propulsão (Ponte H Dupla L298N) - Motores DC em rotação oposta
void ligarPropulsao() {
  digitalWrite(MOTOR1_PIN1, HIGH);
  digitalWrite(MOTOR1_PIN2, LOW);
  digitalWrite(MOTOR2_PIN1, LOW);  
  digitalWrite(MOTOR2_PIN2, HIGH);
}
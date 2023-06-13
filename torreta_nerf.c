#include <stdio.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <Ultrasonic.h>

// Definições necessárias
#define HIGH 1
#define LOW 0
#define ON 1
#define OFF 0
#define TRUE 1
#define FALSE 0

// Definição dos pinos utilizados pelos componentes
#define SERVO_HORIZONTAL_PIN 3
#define SERVO_VERTICAL_PIN 5
#define LASER_PIN 7
#define MOTOR1_PIN1 12
#define MOTOR1_PIN2 13
#define MOTOR2_PIN1 14
#define MOTOR2_PIN2 15
#define SERVO_EMPURRAR_PIN 11
// sensor de distância
#define TRIG_PIN 2
#define ECHO_PIN 3
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

// Estado do disparo
bool DISPARO = OFF;

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

// Inicializa o objeto Ultrasonic
Ultrasonic ultrasonic(TRIG_PIN, ECHO_PIN);

// Conecta o pino TX do HC05 ao pino RX 2 do arduino e pino RX do HC05 ao pino TX 3 do arduino por um divisor de tensão
SoftwareSerial BTserial(RX, TX);

char command = ' ';
 
// Inicializa a variável de verificação da conexão bluetooth
boolean BTconnected = FALSE;

// Variáveis de controle da máquina de estados
State currentState = STATE_IDLE;

// Variáveis temporais para verificação 
unsigned long tempoAtual;     // Variável para armazenar o tempo atual
unsigned long ultimoEnvio;    // Variável para armazenar o tempo do último envio
const unsigned long intervaloEnvio = 1000; // Intervalo de 1s para envio
unsigned long ultimaLeitura; // Variável para armazenar o tempo da última leitura de distância
const unsigned long intervaloLeitura = 500; // Intervalo de 500ms para leitura

void setup() {
  // Anexa servos aos pinos
  servoHorizontal.attach(SERVO_HORIZONTAL_PIN);
  servoVertical.attach(SERVO_VERTICAL_PIN);
  servoEmpurrar.attach(SERVO_EMPURRAR_PIN);

  // Inicialização da posição dos servos
  recoil_servo.write(RECOIL_REST);
  servoHorizontal.write(90);
  //servoVertical.write(tilt_limit_2);
  delay(1000);
  //servoVertical.write(tilt_limit_2 + abs((tilt_limit_2 - tilt_limit_1)/2));
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
  ultimaLeitura = tempoAtual; // Inicializa a variável ultimaLeitura (distância)
}

void loop() {
  // Leitura dos comandos da interface (Python/Bluetooth)
  // (Você precisará adicionar a lógica de leitura dos comandos aqui)

  getDataFromPC(); // recebe o buffer de dados
  if (data_received) {
    move_servo();
    set_recoil();
    
    if (LASER) {
      ligarLaser();   
    }
    else {
      desligarLaser();
    }   

    if (PROPULSION) {
      ligarPropulsao();
    }
    else {
      desligarPropulsao();
    }
  }

  if (DISPARO) {
    fire();
  }
  DISPARO  = FALSE;

  tempoAtual = millis();  // Atualiza o tempo atual a cada iteração do loop

  // faz a leitura do sensor de distância para exibir na interface
  if (tempoAtual - ultimaLeitura >= intervaloLeitura) {
    // Realiza a leitura do sensor de distância
    float distancia_cm = ultrasonic.read();

    // Envia a distância no monitor serial bluetooth
    BTserial.print("Distancia: ");
    BTserial.print(distancia_cm);
    BTserial.println(" cm");

    ultimaLeitura = tempoAtual; // Atualiza o tempo da última leitura
  }
  
  // Máquina de estados
  switch (currentState) {
    case STATE_IDLE:
      // Ação: Mantém a torreta parada em espera e reseta tudo
      bool LASER = OFF;       // Desliga laser  
      bool PROPULSION = OFF;  // Desliga propulsores
      servoHorizontal.write(90);  // Posição inicial
      servoVertical.write(105);    // Posição inicial
      
      // Transição para o próximo estado
      if (bluetoothConectado()) {
        currentState = STATE_BT_CON;
      }
      break;
      
    case STATE_BT_CON:
      // Ação: Mantém a torreta parada
      bool LASER = OFF;       // Desliga laser  
      bool PROPULSION = OFF;  // Desliga propulsores

      if (BTserial.available()) {
        command = BTserial.read();

        // Verificar o comando recebido e realizar ação correspondente
        if (inputBuffer[4] == TRUE) { // botão do laser pressionado
          // Executar ação correspondente ao comando 'laser'
          bool LASER = OFF;       // Desliga laser
        } 
        else if (inputBuffer[2] == TRUE) { // botão de propulsão pressionado
          // Executar ação correspondente ao comando 'motor_dc'
          bool PROPULSION = OFF;  // Desliga propulsores
        } 
      }

      // Transições para outros estados
      if (!bluetoothConectado()) { // Verificação da conexão Bluetooth
        currentState = STATE_IDLE;
      } 
      else if (laserLigado() && !propulsionLigada()) {
        currentState = STATE_LASER_ON;
      } 
      else if (!laserLigado() && propulsionLigada()) {
        currentState = STATE_DC_ON;
      } 
      break;
      
    case STATE_LASER_ON:
      // Ação: Ativa o laser
      bool LASER = ON;       // Desliga laser  
      bool PROPULSION = OFF;  // Desliga propulsores

      if (BTserial.available()) {
        command = BTserial.read();

        // Verificar o comando recebido e realizar ação correspondente
        if (inputBuffer[4] == TRUE) { // botão do laser pressionado
          // Executar ação correspondente ao comando 'laser'
          bool LASER = OFF;       // Desliga laser
        } 
        else if (inputBuffer[2] == TRUE) { // botão de propulsão pressionado
          // Executar ação correspondente ao comando 'motor_dc'
          bool PROPULSION = OFF;  // Desliga propulsores
        } 
      }
      
      // Transições para outros estados
      if (!bluetoothConectado()) {
        currentState = STATE_IDLE;
      } 
      else if (!laserLigado()) {
        currentState = STATE_BT_CON;
      } 
      else if (propulsionLigada()) {
        currentState = STATE_LASER_DC_ON;
      } 
      break;
      
    case STATE_DC_ON:
      // Ação: Ativa a propulsão dos dardos
      bool LASER = OFF;       // Desliga laser  
      bool PROPULSION = ON;  // Desliga propulsores

      if (BTserial.available()) {
        command = BTserial.read();

        // Verificar o comando recebido e realizar ação correspondente
        if (inputBuffer[4] == TRUE) { // botão do laser pressionado
          // Executar ação correspondente ao comando 'laser'
          bool LASER = OFF;       // Desliga laser
        } 
        else if (inputBuffer[2] == TRUE) { // botão de propulsão pressionado
          // Executar ação correspondente ao comando 'motor_dc'
          bool PROPULSION = OFF;  // Desliga propulsores
        } 
        else if (inputBuffer[3] == TRUE) { // botão de disparo pressionado
          // Executar ação correspondente ao comando 'shoot'
          DISPARO = TRUE;
        } 
      }
      
      // Transições para outros estados
      if (!bluetoothConectado()) {
        currentState = STATE_IDLE;
      } 
      else if (!propulsionLigada()) {
        currentState = STATE_BT_CON;
      } 
      else if (laserLigado()) {
        currentState = STATE_LASER_DC_ON;
      }
      break;
      
    case STATE_LASER_DC_ON:
      // Ação: Ativa o laser e a propulsão dos dardos
      bool LASER = ON;       // Desliga laser  
      bool PROPULSION = ON;  // Desliga propulsores
      
      if (BTserial.available()) {
        command = BTserial.read();

        // Verificar o comando recebido e realizar ação correspondente
        if (inputBuffer[4] == TRUE) { // botão do laser pressionado
          // Executar ação correspondente ao comando 'laser'
          bool LASER = OFF;       // Desliga laser
        } 
        else if (inputBuffer[2] == TRUE) { // botão de propulsão pressionado
          // Executar ação correspondente ao comando 'motor_dc'
          bool PROPULSION = OFF;  // Desliga propulsores
        } 
        else if (inputBuffer[3] == TRUE) { // botão de disparo pressionado
          // Executar ação correspondente ao comando 'shoot'
          DISPARO = TRUE;
        } 
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

void getDataFromPC() {
  //expected structure of data [start byte, pan amount, tilt amount, motor on, firing button pressed, laser on, end byte]
  //start byte = 255
  //pan amount = byte between 0 and 253
  //tilt amount = byte between 0 and 253
  //motor on = 0 for off, 1 on
  //laser on = 0 for off, 1 on
  //firing button pressed = 0 for not pressed, 1 for pressed
  //end byte = 254

  if (Serial.available()) {  // If data available in serial

    byte_from_app = Serial.read();   //read the next character available

    if (byte_from_app == 255) {     // look for start byte, if found:
      bytesRecvd = 0;                   //reset byte received to 0(to start populating inputBuffer from start)
      data_received = false;
    }

    else if (byte_from_app == 254) {    // look for end byte, if found:
      data_received = true;                // set data_received to true so the data can be used
    }

    else {                            // add received bytes to buffer
      inputBuffer[bytesRecvd] = byte_from_app;     //add character to input buffer
      bytesRecvd++;                                // increment byte received (this act as an index)
      if (bytesRecvd == buffSize) {    // just a security in case the inputBuffer fills up (shouldn't happen)
        bytesRecvd = buffSize - 1;    // if bytesReceived > buffer size set bytesReceived smaller than buffer size
      }
    }
  }
}

// Lógica para verificar se o Bluetooth está conectado
bool bluetoothConectado() {
  BT_state_previous = BTconnected;
  if (digitalRead(BT_PIN) == HIGH) { // se bluetooth estiver conectado
    BTconnected = TRUE;
    if (BT_state_previous == FALSE) { // inicia BTserial se ele já não estava ligado antes
      BTserial.begin(9600); 
    }
  }
  else {
    BTconnected = FALSE;
  }
  return BTconnected;
}

// Executa o movimento dos servos
void move_servo() {
  byte servoHorizontal_position = map(inputBuffer[0], 0, 253, PAN_LIMIT_2, PAN_LIMIT_1); // converte o valor do inputbuffer na posição do servo
  servoHorizontal.write(servoHorizontal_position); // ajusta a posição do servo horizontal
  byte servoVertical_position = map(inputBuffer[1], 0 , 253, TILT_LIMIT_1, TILT_LIMIT_2); // converte o valor do inputbuffer na posição do servo
  servoVertical.write(servoVertical_position); // ajusta a posição do servo vertical
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

// Desliga o laser
void desligarLaser() {
  digitalWrite(LASER_PIN, LOW);
}

// Liga a propulsão (Ponte H Dupla L298N) - Motores DC em rotação oposta
void ligarPropulsao() {
  digitalWrite(MOTOR1_PIN1, HIGH);
  digitalWrite(MOTOR1_PIN2, LOW);
  digitalWrite(MOTOR2_PIN1, LOW);  
  digitalWrite(MOTOR2_PIN2, HIGH);
}

// Desliga a propulsão (Ponte H Dupla L298N)
void desligarPropulsao() {
  digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, LOW);
  digitalWrite(MOTOR2_PIN1, LOW);
  digitalWrite(MOTOR2_PIN2, LOW);
}

// Aciona o servo que empurra o dardo para o propulsor
void set_recoil() {
  if (inputBuffer[3] == 1) {        //if fire button pressed
    if (!is_firing && !recoiling) { //and not already firing or recoiling
      can_fire = true;              //set can fire to true (see effect in void fire())
    }
  }
  else {                  // if fire button not pressed
    can_fire = false;     //set can fire to false (see effect in void fire())
  }
}

// Realiza o disparo
void Disparar() {
  if (can_fire && !is_firing && PROPULSION) {
    firing_start_time = millis();
    recoil_start_time = millis();
    is_firing = true;
  }

  firing_current_time = millis();
  recoil_current_time = millis();

  if (is_firing && firing_current_time - firing_start_time < firing_time) {
    recoil_servo.write(recoil_pushed);
  }
  else if (is_firing && recoil_current_time - recoil_start_time < recoil_time) {
    recoil_servo.write(recoil_rest);
  }
  else if (is_firing && recoil_current_time - recoil_start_time > recoil_time) {
    is_firing = false;
  }
}
// #include <SoftwareSerial.h>
#include <Servo.h>
#include <Ultrasonic.h>

// Definição dos pinos utilizados pelos componentes
#define SERVO_HORIZONTAL_PIN 4
#define SERVO_VERTICAL_PIN 3
#define SERVO_EMPURRAR_PIN 2
#define LASER_PIN 8
#define MOTOR1_PIN1 9
#define MOTOR1_PIN2 10
#define MOTOR2_PIN1 11
#define MOTOR2_PIN2 12
// sensor de distância
#define TRIG_PIN 22
#define ECHO_PIN 24
// módulo HC05
#define RX 19
#define TX 18
#define BT_PIN 5
// buzzer
#define BUZZER_PIN 13

#define OFF 0
#define ON 1
#define FALSE '0'
#define TRUE '1'

// Definições de movimento
#define PAN_LIMIT_1 10
#define PAN_LIMIT_2 180
#define TILT_LIMIT_1 60
#define TILT_LIMIT_2 150
#define RECOIL_REST 150 // Ângulo do servo na posição de descanso
#define RECOIL_PUSHED 80 // Ângulo do servo necessário para empurrar o dardo

// Variáveis e definições associadas aos dados de comunicação serial
#define buffSize 40
#define startMarker 255
#define endMarker 254
byte receivedByte;
byte inputBuffer[buffSize];
boolean data_received = false;

#define MAX_SIZE 40 // Tamanho máximo do vetor de dados
int dataIndex = 0; // Índice de posição atual no vetor de dados

boolean started = false; // Flag indicando se o marcador de início foi encontrado

// Variáveis associadas aos módulos de propulsão e tempo de disparo
bool is_firing = false;
bool can_fire = false;
bool recoiling = false;

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

// Inicializa o objeto Ultrasonic
Ultrasonic ultrasonic(TRIG_PIN, ECHO_PIN);

// Conecta o pino TX do HC05 ao pino RX 2 do arduino e pino RX do HC05 ao pino TX 3 do arduino por um divisor de tensão
// SoftwareSerial BTserial(RX, TX);
 
// Inicializa a variável de verificação da conexão bluetooth
boolean BTconnected = false;

// Inicializa variável de controle da máquina de estados
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
  servoEmpurrar.write(RECOIL_REST);
  delay(1000);
  servoHorizontal.write(90);
  delay(1000);
  servoVertical.write(90);

  // Define pino do laser
  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, OFF);

  // Define pino dos motores DC
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
  pinMode(MOTOR2_PIN1, OUTPUT);
  pinMode(MOTOR2_PIN2, OUTPUT);

  // Define pino do bluetooth
  pinMode(BT_PIN, INPUT); 

  // Define pino do buzzer
  pinMode(BUZZER_PIN, OUTPUT);

  // Comunicação serial
  Serial.begin(9600);
  Serial1.begin(9600);

  // Inicializações de tempo
  tempoAtual = millis();     // Inicializa a variável tempoAtual
  ultimoEnvio = tempoAtual;  // Inicializa a variável ultimoEnvio (bluetooth)
  ultimaLeitura = tempoAtual; // Inicializa a variável ultimaLeitura (distância)

  beep(); // beep simples ao iniciar o sistema
}

void loop() {
  getData(); // Executa funçao que recebe o buffer de dados

  // if (data_received) {
  //       // Verificar o comando recebido e realizar ação correspondente
  //       if (byte_from_app == 1) { // botão do laser ativado
  //         // Executar ação correspondente ao comando 'laser'
  //         LASER = ON;       // Liga laser
  //         ligarLaser();
  //       } else {
  //         LASER = OFF;
  //         desligarLaser();
  //       }
  // }

  tempoAtual = millis();  // Atualiza o tempo atual a cada iteração do loop

  // faz a leitura do sensor de distância para exibir na interface
  // if (tempoAtual - ultimaLeitura >= intervaloLeitura) {
  //   // Realiza a leitura do sensor de distância
  //   float distancia_cm = ultrasonic.read();

  //   // Envia a distância no monitor serial bluetooth
  //   Serial.print("Distancia: ");
  //   Serial.print(distancia_cm);
  //   Serial.println(" cm");

  //   Serial1.print("Distancia: ");
  //   Serial1.print(distancia_cm);
  //   Serial1.println(" cm");

  //   ultimaLeitura = tempoAtual; // Atualiza o tempo da última leitura
  // }
  
  // Máquina de estados
  switch (currentState) {
    case STATE_IDLE:
      // Ação: Mantém a torreta parada em espera e reseta tudo
      LASER = OFF;       // Desliga laser  
      PROPULSION = OFF;  // Desliga propulsores
      servoHorizontal.write(90);  // Reseta posição inicial
      delay(300);
      servoVertical.write(105);    // Reseta posição inicial
      delay(300);
      
      // Transição para o próximo estado
      if (bluetoothConectado()) { 
        currentState = STATE_BT_CON;
        double_beep(); // beep duplo para avisar conexão
      }
      break;
      
    case STATE_BT_CON:
      // Ação: Mantém a torreta parada com BT conectado

      LASER = OFF;       // Desliga laser  
      PROPULSION = OFF;  // Desliga propulsores

      move_servo(); // Ajusta posição do servoHorizontal e servoVertical

      if (data_received) {
          // Serial.println(sizeof(inputBuffer[4]));
        // Verificar o comando recebido e realizar ação correspondente
        if (inputBuffer[4] == 1) { // botão do laser ativado
          // Executar ação correspondente ao comando 'laser'
          LASER = ON;       // Liga laser
          ligarLaser();
        } 
        else if (inputBuffer[2] == 1) { // botão de propulsão ativado
          // Executar ação correspondente ao comando 'motor_dc'
          PROPULSION = ON;  // Liga propulsores
          ligarPropulsao();
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
      LASER = ON;       // Liga laser  
      PROPULSION = OFF;  // Desliga propulsores

      move_servo(); // Ajusta posição do servoHorizontal e servoVertical

      if (data_received) {
        // Verificar o comando recebido e realizar ação correspondente
        if (inputBuffer[4] == 0) { // botão do laser desativado
          // Executar ação correspondente ao comando 'laser'
          LASER = OFF;       // Desliga laser
          desligarLaser();
        } 
        if (inputBuffer[2] == 1) { // botão de propulsão ativado
          // Executar ação correspondente ao comando 'motor_dc'
          PROPULSION = ON;  // Liga propulsores
          ligarPropulsao();
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
      LASER = OFF;       // Desliga laser  
      PROPULSION = ON;  // Liga propulsores

      move_servo(); // Ajusta posição do servoHorizontal e servoVertical

      if (data_received) {
        // Verificar o comando recebido e realizar ação correspondente
        if (inputBuffer[4] == 1) { // botão do laser ativado
          // Executar ação correspondente ao comando 'laser'
          LASER = ON;       // Liga laser
          ligarLaser();
        } 
        if (inputBuffer[2] == 0) { // botão de propulsão desativado
          // Executar ação correspondente ao comando 'motor_dc'
          PROPULSION = OFF;  // Desliga propulsores
          desligarPropulsao();
        } 
        if (inputBuffer[3] == 1) { // botão de disparo pressionado
          // Executar ação correspondente ao comando 'shoot'
          if (!is_firing && !recoiling) { // não está atirando nem recuando servoEmpurrar
            can_fire = true;              // habilita disparo (efeitos na função Disparar())
          }
        }
        else {                  // se o botão de disparo não estiver pressionado
          can_fire = false;     // desabilita disparo (efeitos na função Disparar())
        }
        Disparar();           
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
      LASER = ON;       // Liga laser  
      PROPULSION = ON;  // Liga propulsores

      move_servo(); // Ajusta posição do servoHorizontal e servoVertical

      if (data_received) {
        // Verificar o comando recebido e realizar ação correspondente
        if (inputBuffer[4] == 0) { // botão do laser desativado
          // Executar ação correspondente ao comando 'laser'
          LASER = OFF;       // Desliga laser
          desligarLaser();
        } 
        else if (inputBuffer[2] == 0) { // botão de propulsão desativado
          // Executar ação correspondente ao comando 'motor_dc'
          PROPULSION = OFF;  // Desliga propulsores
          desligarPropulsao();
        } 
        if (inputBuffer[3] == 1) { // botão de disparo pressionado
          // Executar ação correspondente ao comando 'shoot'
          if (!is_firing && !recoiling) { // não está atirando nem recuando servoEmpurrar
            can_fire = true;              // habilita disparo (efeitos na função Disparar())
          }
        }
        else {                  // se o botão de disparo não estiver pressionado
          can_fire = false;     // desabilita disparo (efeitos na função Disparar())
        }
        Disparar(); 
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

// void getData() {
//   //expected structure of data [start byte, pan amount, tilt amount, motor on, firing button pressed, laser on, end byte]
//   //start byte = 255
//   //pan amount = byte between 0 and 253
//   //tilt amount = byte between 0 and 253
//   //motor on = 0 for off, 1 on
//   //laser on = 0 for off, 1 on
//   //firing button pressed = 0 for not pressed, 1 for pressed
//   //end byte = 254



void getData() {
  if (Serial1.available()) {
    receivedByte = Serial1.read(); // Lê o caractere disponível    

    if (receivedByte == 255) { // Verifica se é o marcador de início
      data_received = false;
      dataIndex = 0; // Reseta o índice de posição no vetor de dados

    } else if (receivedByte == 254) { // Verifica se é o marcador de fim
      data_received = true;

    } else { // Se o marcador de início foi encontrado, armazena os dados
      inputBuffer[dataIndex++] = receivedByte;

      if (dataIndex >= MAX_SIZE) { // Evita estouro de buffer
        dataIndex = MAX_SIZE - 1;
      }
    }
  }
}

// Lógica para verificar se o Bluetooth está conectado
bool bluetoothConectado() {
  bool BT_state_previous = BTconnected;
  if (digitalRead(BT_PIN) == HIGH) { // se bluetooth estiver conectado
    BTconnected = true;

  }
  else {
    BTconnected = false;
  }
  return BTconnected;
}

// Executa o movimento dos servos
void move_servo() {
  byte servoHorizontal_position = map(inputBuffer[0], 0, 253, PAN_LIMIT_1, PAN_LIMIT_2); // converte o valor do inputbuffer na posição do servo
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
  digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, HIGH);
  digitalWrite(MOTOR2_PIN1, HIGH);  
  digitalWrite(MOTOR2_PIN2, LOW);
}

// Desliga a propulsão (Ponte H Dupla L298N)
void desligarPropulsao() {
  digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, LOW);
  digitalWrite(MOTOR2_PIN1, LOW);
  digitalWrite(MOTOR2_PIN2, LOW);
}

// Realiza o disparo
void Disparar() {
  if (can_fire && !is_firing && PROPULSION) { // Se puder atirar, não estiver atirando e motores de propulsão ligados
    firing_start_time = millis();
    recoil_start_time = millis();
    is_firing = true;
  }

  firing_current_time = millis();
  recoil_current_time = millis();

  if (is_firing && firing_current_time - firing_start_time < firing_time) {
    servoEmpurrar.write(RECOIL_PUSHED);
  }
  else if (is_firing && recoil_current_time - recoil_start_time < recoil_time) {
    servoEmpurrar.write(RECOIL_REST);
  }
  else if (is_firing && recoil_current_time - recoil_start_time > recoil_time) {
    is_firing = false;
  }
}

// Ligar o buzzer
void beep(){
  int freq = 2000;
  tone(BUZZER_PIN, freq); // gera frequencia de 2000Hz no buzzer
  delay(300);           // aguarda 0,3s
  noTone(BUZZER_PIN); // para frequencia no buzzer
}

// Ligar o buzzer duplo
void double_beep(){
  int freq = 2000;
  tone(BUZZER_PIN, freq); // gera frequencia de 2000Hz no buzzer
  delay(300);           // aguarda 0,3s
  noTone(BUZZER_PIN); // para frequencia no buzzer
  delay(100);           // aguarda 0,1s
  tone(BUZZER_PIN, freq); // gera frequencia de 2000Hz no buzzer
  delay(300);           // aguarda 0,3s
  noTone(BUZZER_PIN); // para frequencia no buzzer
}
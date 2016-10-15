#include <Servo.h>

/*
 * Sirius
 * ----------------------------------------
 * Robô projetado para superar os desafoios da OBR 2016
 * ATENÇÃO: o código não está finalizado e foi descontinuado
 * ----------------------------------------
 * Robot designed to solve OBR 2016 objectives
 * WARNING: the following code isn't finished and has been discontinued
 * ----------------------------------------
 * Author: https://github.com/rafaelalmeidatk
*/

//--------------------------------------------------
// Servos

Servo servos[2];

const int SV_PIN_LEFT = 10; // 0: 94
const int SV_PIN_RIGHT = 9; // 1: 96

const int SV_LEFT = 0;
const int SV_RIGHT = 1;

//--------------------------------------------------
// Claw Servos

Servo clawServos[2];

const int CWSRV_PIN_LEFT = 6;
const int CWSRV_PIN_RIGHT = 7;

const int HEIGHT = 0;
const int CLAW = 1;

const int OPEN = 0;
const int CLOSE = 1;
const int GO_UP = 2;
const int GO_DOWN = 3;
const int INIT_H = 4;

//--------------------------------------------------
// IR Sensors

#define BLACK_VALUE             700
#define SILVER_VALUE            600
#define NUM_SENSORS             6

unsigned int sensorPins[NUM_SENSORS] = { A0, A1, A2, A3, A4, A5 };
unsigned int sensorValues[NUM_SENSORS];
long sensorLastPositionValue = 0;
boolean testedOnGap;

//--------------------------------------------------
// Color sensors

#define USE_COLOR_SENSORS        true

#define RED_MINIMUM                  50
#define GREEN_MINIMUM                40
#define BLUE_MINIMUM                 40

#define RED_MINIMUM_RIGHT            50
#define GREEN_MINIMUM_RIGHT          30
#define BLUE_MINIMUM_RIGHT           30

#define RED_GREEN_DIFFERENCE         10

const int RED = 0;
const int GREEN = 1;
const int BLUE = 2;
const int CS_ESQ = 0;
const int CS_DIR = 1;
const int S0 = 0;
const int S1 = 1;
const int S2 = 2;
const int S3 = 3;
const int OUT = 4;

unsigned int colorSensors[2][5] = {
  {27, 29, 25, 23, 24},
  {51, 50, 49, 48, 47}
};

boolean gotGreen[2] = { false, false };
boolean gotSilverTape[2] = { false, false };
boolean needTurnLeft = false;
boolean needTurnRight = false;

//--------------------------------------------------
// Buttons

const int RIGHT_BUTTON_PIN = 3;
const int LEFT_BUTTON_PIN = 4;
const int OBSTC_BUTTON_PIN = 5;

//--------------------------------------------------
// Ultrasonic Sensors

const int USSR_UP = 0;
const int USSR_DOWN = 1;

const int ULTRA_SR_TRIG_PIN = 40;
const int ULTRA_SR_ECHO_PIN = 38;

const int ULTRA_SR_UP_TRIG_PIN = 31;
const int ULTRA_SR_UP_ECHO_PIN = 30;

//--------------------------------------------------
// Moves

const int FRONT = 0;
const int BACK = 1;
const int LEFT = 2;
const int RIGHT = 3;
const int STOP = 4;

int moves[2][9] = {
  {0, 255, 255, 0, 100 },
  {255, 0, 255, 0, 94 }
};

const int LEFT_90 = 2200;
const int RIGHT_90 = 2400;

//--------------------------------------------------
// Rescue mode

bool rescueMode = false;
bool found = false;
bool forceFound = false;

//--------------------------------------------------
// Calibration
// Mudar o valor para true irá ativar o modo de calibração
// Uso através do Serial Monitor

#define CALIBRATION              false

//--------------------------------------------------

//--------------------------------------------------
// * moveRobot
// * Manda um pulso PWM para cada servo dependendo do moveId

void moveRobot(int moveId) {
  servos[SV_LEFT].write(moves[SV_LEFT][moveId]);
  servos[SV_RIGHT].write(moves[SV_RIGHT][moveId]);
}

//--------------------------------------------------
// * clawMove
// * Manda um pulso PWM para cada servo da garra dependendo do cMove

void clawMove(int cMove) {
  switch (cMove) {
    case OPEN:
      clawServos[CLAW].write(90);
      break;
    case CLOSE:
      clawServos[CLAW].write(0);
      break;
    case GO_UP:
      clawServos[HEIGHT].write(170);
      break;
    case GO_DOWN:
      clawServos[HEIGHT].write(50);
      break;
    case INIT_H:
      clawServos[HEIGHT].write(130);
      break;
  }
}

//--------------------------------------------------
// * clawInit
// * Move a garra para as posições iniciais

void clawInit() {
  clawMove(OPEN);
  clawMove(INIT_H);
}

//--------------------------------------------------
// * readUltrasonicDistance
// * Retorna a distância de um sensor ultrasonico (dependendo do sensorId) em centímetros aproximados

int readUltrasonicDistance(int sensorId) {
  unsigned long pulseWidth = 0;
  int trigPin = sensorId == USSR_UP ? ULTRA_SR_UP_TRIG_PIN : ULTRA_SR_TRIG_PIN;
  int echoPin = sensorId == USSR_UP ? ULTRA_SR_UP_ECHO_PIN : ULTRA_SR_ECHO_PIN;

  if (sensorId == USSR_UP) {
    digitalWrite(trigPin, LOW);
    digitalWrite(trigPin, HIGH);
    pulseWidth = pulseIn(echoPin, LOW);
  } else if (sensorId == USSR_DOWN) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    pulseWidth = pulseIn(echoPin, HIGH);
  }

  return (pulseWidth >= 10200 ? 0 : pulseWidth / 29 / 2);
}

//--------------------------------------------------
// * readIRSensors
// * Executa a leitura de todos os sensores infravermelhos e guarda seus respectivos valores em um array para uso posterior

void readIRSensors(boolean debug) {
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
    if (sensorValues[i] > BLACK_VALUE) {
      testedOnGap = false;
    }
    if (debug) {
      Serial.print(sensorValues[i]);
      Serial.print('\t');
    }
  }
}

//--------------------------------------------------
// * inRange
// * Retorna true caso o "value" estiver igual ao "equalsTo" considerando uma margem "margin" tanto para cima quanto para baixo

boolean inRange(int value, int equalsTo, int margin) {
  return equalsTo >= (value - margin) && (value + margin) >= equalsTo;
}

//--------------------------------------------------
// * readColorSensors
// * Executa a leitura dos sensores de cor, o sucesso na detecção do verde é guardado em um array para uso posterior

void readColorSensors(boolean debug, int start, int maxIndex) {
  if (!USE_COLOR_SENSORS) {
    return;
  }
  for (int i = start; i < maxIndex; i++) {
    int red = 0, green = 0, blue = 0;
    int out = colorSensors[i][OUT];

    digitalWrite(colorSensors[i][S2], LOW);
    digitalWrite(colorSensors[i][S3], LOW);
    red = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);

    digitalWrite(colorSensors[i][S3], HIGH);
    blue = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);

    digitalWrite(colorSensors[i][S2], HIGH);
    green = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);

    int redMinimum = i == CS_DIR ? RED_MINIMUM_RIGHT : RED_MINIMUM;
    int greenMinimum = i == CS_DIR ? GREEN_MINIMUM_RIGHT : GREEN_MINIMUM;
    int blueMinimum = i == CS_DIR ? BLUE_MINIMUM_RIGHT : BLUE_MINIMUM;
    int redGreenDifference = red - green;

    if (red > green && red > blue && red > redMinimum && green > greenMinimum && blue > blueMinimum && redGreenDifference > RED_GREEN_DIFFERENCE) {
      Serial.print("GOT GREEN!");
      Serial.println(i);
      gotGreen[i] = true;
    } else {
      gotGreen[i] = false;
    }

    if (debug) {
      Serial.print("R: ");
      Serial.print(red, DEC);
      Serial.print(" G: ");
      Serial.print(green, DEC);
      Serial.print(" B: ");
      Serial.print(blue, DEC);
      Serial.print('\t');
      Serial.print("Red & Green difference: ");
      Serial.println(redGreenDifference);
      delay(100);
    }
  }
  /**/
}

//--------------------------------------------------
// * readAllSensors
// * Executa a leitura de todos os sensores IR e sensores de cor

void readAllSensors() {
  readIRSensors(false);
  readColorSensors(false, 0, 2);
}

//--------------------------------------------------
// * calculatePosition
// * Retorna a posição do robô na linha com base nos valores dos sensores IR

int calculatePosition() {
  boolean onLine = false;
  long avg = 0;
  long sum = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    int value = sensorValues[i];

    if (value > 200) {
      onLine = true;
    }

    if (value > 50) {
      avg += (long)(value) * (i * 1000);
      sum += value;
    }
  }

  if (!onLine) {
    if (sensorLastPositionValue < (NUM_SENSORS - 1) * 1000 / 2) {
      return 0;
    } else {
      return (NUM_SENSORS - 1) * 1000;
    }
  }

  sensorLastPositionValue = avg / sum;
  return sensorLastPositionValue;
}

//--------------------------------------------------
// * isBlack
// * Retorna true caso o sensor de index "sensorIndex" esteja com um valor acima do calibrado para o preto

boolean isBlack(int sensorIndex) {
  return sensorValues[sensorIndex] > BLACK_VALUE;
}

//--------------------------------------------------
// * isCenterBlack
// * Retorna true caso o módulo da posição menos o erro seja maior do que 1000

boolean isCenterBlack() {
  int error = calculatePosition() - 2500;
  return error > -1000 && error < 1000;
}

//--------------------------------------------------
// * got90Right
// * Retorna true caso os sensores da direita estejam detectando preto

boolean got90Right() {
  return isBlack(3) && isBlack(4) && isBlack(5);
}

//--------------------------------------------------
// * got90Left
// * Retorna true caso os sensores da esquerda estejam detectando preto

boolean got90Left() {
  return isBlack(0) && isBlack(1) && isBlack(2);
}

//--------------------------------------------------
// * isAllBlack
// * Retorna true caso todos os sensores IR estejam retornando preto

boolean isAllBlack() {
  return isBlack(0) && isBlack(1) && isBlack(2) && isBlack(3) && isBlack(4) && isBlack(5);
}

//--------------------------------------------------
// * isAllWhite
// * Retorna true caso todos os sensores IR não estejam retornando preto

boolean isAllWhite() {
  return !isBlack(0) && !isBlack(1) && !isBlack(2) && !isBlack(3) && !isBlack(4) && !isBlack(5);
}

//--------------------------------------------------
// * performObstacleEvade
// * Executa a rotina de desvio de obstáculos

void performObstacleEvade() {
  clawMove(CLOSE);
  moveRobot(BACK);
  delay(700);
  moveRobot(LEFT);
  delay(1700);
  moveRobot(FRONT);
  delay(2800);
  moveRobot(RIGHT);
  delay(1700);
  moveRobot(FRONT);
  delay(2500);
  moveRobot(RIGHT);
  delay(400);
  moveRobot(FRONT);
  delay(2500);
  moveRobot(RIGHT);
  delay(2200);
  moveRobot(FRONT);
  delay(200);
  clawMove(OPEN);

  while (true) {
    readIRSensors(false);
    if (isBlack(2) && isBlack(3)) {
      break;
    }
  }

  moveRobot(FRONT);
  delay(500);
  moveRobot(LEFT);
  delay(1800);
  moveRobot(BACK);
  delay(1700);
}

//--------------------------------------------------
// * getMinDistance
// * Retorna o menor valor dentro de um array (de tamanho 100) de inteiros.

int getMinDistance(int distances[100]) {
  int minDist = -1;
  for (int i = 0; i < 100; i++) {
    if ((distances[i] != 0 && distances[i] < minDist) || minDist < 0) {
      minDist = distances[i];
    }
  }
  return minDist;
}

//--------------------------------------------------
// * readDoubleUltrassonicDistanceWithFilter
// * Lê os dois valores de ambos os sensores ultrasonicos e retorna true caso ambos os dados tenham passado pelo filtro

int readDoubleUltrassonicDistanceWithFilter() {
  unsigned int distanceUp = readUltrasonicDistance(USSR_UP);
  delay(10);
  unsigned int distanceDown = readUltrasonicDistance(USSR_DOWN);
  Serial.print(distanceUp);
  Serial.print('\t');
  Serial.println(distanceDown);
  int difference = abs(distanceUp - distanceDown);
  return (difference > 30 && distanceDown < 50) ? distanceDown : 0;
}

//--------------------------------------------------
// * foundBall
// * Retorna true caso o valor retornado pela função chamada seja maior que zero

boolean foundBall() {
  return readDoubleUltrassonicDistanceWithFilter() > 0;
}

//--------------------------------------------------
// * searchTriangle
// * Executa a rotina de busca pelo triângulo

void searchTriangle() {
  moveRobot(FRONT);
  bool jump = false;
  int side = 0;
  while (true) {
    if (digitalRead(OBSTC_BUTTON_PIN) == HIGH) {
      break;
    }
    if (digitalRead(RIGHT_BUTTON_PIN) == HIGH) {
      side = 0;
      jump = true;
      break;
    }
    if (digitalRead(LEFT_BUTTON_PIN) == HIGH) {
      side = 1;
      jump = true;
      break;
    }
  }
  if (!jump) {
    moveRobot(BACK);
    delay(1000);
    moveRobot(RIGHT);
    delay(2400);
    moveRobot(FRONT);
  }
  while (true) {
    if (jump) {
      break;
    }
    if (digitalRead(RIGHT_BUTTON_PIN) == HIGH) {
      side = 0;
      break;
    } else if (digitalRead(LEFT_BUTTON_PIN) == HIGH) {
      side = 1;
      break;
    } else if (digitalRead(OBSTC_BUTTON_PIN) == HIGH) {
      moveRobot(BACK);
      delay(1000);
      moveRobot(RIGHT);
      delay(2400);
      moveRobot(FRONT);
      break;
    }
  }
  moveRobot(BACK);
  delay(1000);
  moveRobot(side == 0 ? RIGHT : LEFT);
  delay(4300);
  moveRobot(BACK);
  delay(3000);
  clawMove(OPEN);
  delay(1500);
  moveRobot(FRONT);
  delay(1000);
  found = false;
}

//--------------------------------------------------
// * rescueBall
// * Executa a rotina de resgate em um alvo já encontrado. Retorna true caso a rotina seja executada com sucesso.

boolean rescueBall() {
  bool found = false;
  moveRobot(STOP);
  delay(100);
  moveRobot(FRONT);
  delay(700);
  moveRobot(STOP);
  delay(100);

  unsigned int ballDistance = 100;

  while (ballDistance > 15) {
    if (digitalRead(OBSTC_BUTTON_PIN) == HIGH) {
      moveRobot(FRONT);
      delay(300);
      moveRobot(BACK);
      delay(100);
      moveRobot(RIGHT);
      delay(RIGHT_90);
    }
    int distances[100];
    for (int i = 0; i < 50; i++) {
      if (digitalRead(OBSTC_BUTTON_PIN) == HIGH) {
        moveRobot(FRONT);
        delay(300);
        moveRobot(BACK);
        delay(100);
        moveRobot(RIGHT);
        delay(RIGHT_90);
        break;
      }
      moveRobot(RIGHT);
      distances[i] = readUltrasonicDistance(USSR_DOWN);
      delay(10);
    }
    moveRobot(LEFT);
    delay(700);
    for (int i = 50; i < 100; i++) {
      if (digitalRead(OBSTC_BUTTON_PIN) == HIGH) {
        moveRobot(FRONT);
        delay(300);
        moveRobot(BACK);
        delay(100);
        moveRobot(RIGHT);
        delay(RIGHT_90);
        break;
      }
      distances[i] = readUltrasonicDistance(USSR_DOWN);
      delay(5);
    }
    moveRobot(RIGHT);
    unsigned int minDist = getMinDistance(distances);
    unsigned int currentDist = 0;
    unsigned int startTime = millis() + 3500;
    while (true) {
      if (digitalRead(OBSTC_BUTTON_PIN) == HIGH) {
        moveRobot(FRONT);
        delay(300);
        moveRobot(BACK);
        delay(100);
        moveRobot(RIGHT);
        delay(RIGHT_90);
      }
      if (millis() > startTime) {
        return false;
      }
      currentDist = readUltrasonicDistance(USSR_DOWN);
      if (currentDist != 0 && inRange(currentDist, minDist, 2)) {
        break;
      }
      delay(10);
      if (foundBall())
        found = true;
      delay(10);
    }

    if (!found) {
      return false;
    }

    unsigned int frontTime = millis() + 700;
    moveRobot(FRONT);
    while (millis() < frontTime) {
      ballDistance = readUltrasonicDistance(USSR_DOWN);
      if (ballDistance <= 15) {
        break;
      }
      delay(30);
    }
  }

  moveRobot(STOP);
  delay(100);
  moveRobot(LEFT);
  delay(4400);
  moveRobot(STOP);
  delay(500);
  clawMove(GO_DOWN);
  delay(1000);
  moveRobot(BACK);
  delay(2000);
  clawMove(GO_DOWN);
  clawMove(CLOSE);
  delay(1000);
  moveRobot(STOP);
  clawMove(GO_UP);
  delay(1000);
  moveRobot(LEFT);
  delay(4000);
  return true;
}

//--------------------------------------------------
// * processRescueMode
// * Executa a rotina de procura das vítimas

void processRescueMode() {
  moveRobot(RIGHT);
  unsigned int startTime = millis() + RIGHT_90;
  boolean retry = false;
  found = forceFound;
  while (true) {
    if (!retry && !found && millis() > startTime) {
      break;
    }
    if (foundBall() || forceFound) {
      forceFound = false;
      found = true;
      if (rescueBall()) {
        searchTriangle();
        retry = false;
        return;
      } else {
        moveRobot(RIGHT);
        retry = true;
        found = false;
      }
    } else {
      found = false;
    }
    delay(30);
  }
  if (!found) {
    moveRobot(LEFT);
    delay(LEFT_90);
    startTime = millis() + 2000;
    moveRobot(FRONT);
    while (millis() < startTime) {
      if (digitalRead(OBSTC_BUTTON_PIN) == HIGH || digitalRead(LEFT_BUTTON_PIN) == HIGH) {
        moveRobot(BACK);
        delay(1000);
        moveRobot(RIGHT);
        delay(RIGHT_90);
        moveRobot(FRONT);
        startTime = millis() + 2000;
      }
      if (foundBall()) {
        forceFound = true;
        break;
      } else {
        forceFound = false;
      }
      delay(50);
    }
  }
}

//--------------------------------------------------
// * setup
// * Inicialização do robô

void setup() {
  Serial.begin(9600);

  // Servos setup
  servos[SV_LEFT].attach(SV_PIN_LEFT);
  servos[SV_RIGHT].attach(SV_PIN_RIGHT);

  // Claw Servos setup
  clawServos[HEIGHT].attach(CWSRV_PIN_LEFT);
  clawServos[CLAW].attach(CWSRV_PIN_RIGHT);

  // Color sensors setup
  for (int i = 0; i < 2; i++) {
    pinMode(colorSensors[i][S0],      OUTPUT);
    pinMode(colorSensors[i][S1],      OUTPUT);
    pinMode(colorSensors[i][S2],      OUTPUT);
    pinMode(colorSensors[i][S3],      OUTPUT);
    pinMode(colorSensors[i][OUT],     INPUT);
    digitalWrite(colorSensors[i][S0], HIGH);

    digitalWrite(colorSensors[i][S1], LOW);
  }

  // Ultrasonic sensors setup
  pinMode(ULTRA_SR_TRIG_PIN, OUTPUT);
  digitalWrite(ULTRA_SR_TRIG_PIN, HIGH);
  pinMode(ULTRA_SR_ECHO_PIN, INPUT);

  pinMode(ULTRA_SR_UP_TRIG_PIN, OUTPUT);
  digitalWrite(ULTRA_SR_UP_TRIG_PIN, HIGH);
  pinMode(ULTRA_SR_UP_ECHO_PIN, INPUT);

  // Buttons setup
  pinMode(OBSTC_BUTTON_PIN, INPUT);
  pinMode(LEFT_BUTTON_PIN, INPUT);
  pinMode(RIGHT_BUTTON_PIN, INPUT);

  // Stops the robot
  moveRobot(STOP);

  // Initialize the claw position
  clawInit();
}

//--------------------------------------------------
// * loop
// * Loop principal

void loop() {
  // Calibração
  if (CALIBRATION) {
    char commandByte = '*';

    Serial.println("===== Calibration =====");
    Serial.println("Choose an option");
    Serial.println(" - (I)R Sensors");
    Serial.println(" - (C)olor Sensors");

    while (Serial.available() == 0);
    commandByte = Serial.read();

    if (commandByte == 'i' || commandByte == 'I') {
      commandByte = '*';
      while (commandByte != 'x') {
        readIRSensors(true);
        if (Serial.available()) {
          commandByte = Serial.read();
        }
      }
    } else if (commandByte == 'c' || commandByte == 'C') {
      commandByte = '*';
      while (commandByte != 'x') {
        Serial.println("Choose a side");
        Serial.println(" - (L)eft Sensors");
        Serial.println(" - (R)ight Sensors");
        while (Serial.available() == 0);
        commandByte = Serial.read();
        if (commandByte == 'l' || commandByte == 'L') {
          while (commandByte != 'x') {
            readColorSensors(true, 0, 1);
            if (Serial.available()) {
              commandByte = Serial.read();
            }
          }
          commandByte = '*';
        } else if (commandByte == 'r' || commandByte == 'R') {
          while (commandByte != 'x') {
            readColorSensors(true, 1, 2);
            if (Serial.available()) {
              commandByte = Serial.read();
            }
          }
          commandByte = '*';
        }
      }
    }
    return;
  }

  // Se estiver no modo de resgate, executa a rotina e ignora o resto do código
  if (rescueMode) {
    processRescueMode();
    return;
  }

  // Caso o botão do obstáculo seja acionado, executa a rotina de desvio
  if (digitalRead(OBSTC_BUTTON_PIN) == HIGH) {
    performObstacleEvade();
  }

  // Executa uma leitura dos sensores infravermelhos
  readIRSensors(false);

  // Um possível gap foi encontrado, verifica se é uma fita prateada (para início do modo de resgate)
  if (isAllWhite() && !testedOnGap) {
    testedOnGap = true;
    moveRobot(FRONT);
    delay(30);
    moveRobot(STOP);
    readIRSensors(true);
    int count = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (sensorValues[i] < BLACK_VALUE && inRange(sensorValues[i], SILVER_VALUE, 200)) {
        count++;
      }
    }
    if (count >= 2) {
      moveRobot(FRONT);
      delay(3500);
      moveRobot(STOP);
      rescueMode = true;
      return;
    } else {
      moveRobot(BACK);
      delay(50);
    }
  }

  // Caso esteja tudo preto ou tudo branco e não há a necessidade de virar para um dos lados, siga em frente
  if ((isAllWhite() || isAllBlack()) && (!needTurnLeft && !needTurnRight)) {
    moveRobot(FRONT);
    return;
  // Se não precisa ir para a esquerda e foi detectado um ângulo de 90º 
  } else if (!needTurnLeft && (got90Right() || needTurnRight)) {
    Serial.println("GOT 90 RIGHT!");
    boolean k = true;
    moveRobot(FRONT);
    delay(30);
    moveRobot(STOP);
    delay(100);
    readAllSensors();
    if (!needTurnRight) {
      if (gotGreen[0]) {
        needTurnLeft = true;
        Serial.println("Got green Left going to Right!");
        return;
      } else if (gotGreen[1]) {
        needTurnRight = true;
        Serial.println("Got green Right going to Right!");
      }
    }
    moveRobot(FRONT);
    delay(250);
    readIRSensors(false);
    if (isAllWhite() || needTurnRight) {
      Serial.println("Need to turn");
      if (needTurnRight) {
        moveRobot(FRONT);
        delay(200);
        moveRobot(RIGHT);
        delay(1000);
        moveRobot(FRONT);
        delay(200);
      } else {
        moveRobot(RIGHT);
        delay(500);
      }
      moveRobot(RIGHT);
      while (true) {
        readIRSensors(false);
        if (isBlack(1)) {
          Serial.println("found!");
          break;
        }
      }
      needTurnLeft = false;
      needTurnRight = false;
    } else {
      Serial.println("Continue to front");
    }
    Serial.println("DONE 90 right!");
  } else if (!needTurnRight && (got90Left() || needTurnLeft)) {
    Serial.println("GOT 90 LEFT!");
    boolean k = true;
    moveRobot(FRONT);
    delay(30);
    moveRobot(STOP);
    delay(100);
    readAllSensors();
    if (!needTurnLeft) {
      if (gotGreen[0]) {
        needTurnLeft = true;
        Serial.println("Got green Left going to Lef!");
      } else if (gotGreen[1]) {
        needTurnRight = true;
        Serial.println("Got green Right going to Left!");
        return;
      }
    }
    moveRobot(FRONT);
    delay(250);
    readIRSensors(false);
    if (isAllWhite() || needTurnLeft) {
      Serial.println("Need to turn");
      if (needTurnLeft) {
        moveRobot(FRONT);
        delay(200);
        moveRobot(LEFT);
        delay(1000);
        moveRobot(FRONT);
        delay(200);
      } else {
        moveRobot(LEFT);
        delay(500);
      }
      moveRobot(LEFT);
      while (true) {
        readIRSensors(false);
        if (isBlack(4)) {
          Serial.println("found!");
          break;
        }
      }
      needTurnLeft = false;
      needTurnRight = false;
    } else {
      Serial.println("Continue to front");
    }
    Serial.println("DONE 90 left!");
  }

  int position = calculatePosition();
  int error = position - 2500;

  if (error < -700) {
    moveRobot(LEFT);
    if (error < -900) {
      delay(100);
    }
    if (error < -1000) {
      delay(100);
    }
    if (error < -1200) {
      delay(120);
    }
    if (error < -1300) {
      delay(150);
    }
  } else if (error > 700) {
    moveRobot(RIGHT);
    if (error > 900) {
      delay(100);
    }
    if (error > 1000) {
      delay(100);
    }
    if (error > 1200) {
      delay(120);
    }
    if (error > 1300) {
      delay(150);
    }
  } else {
    moveRobot(FRONT);
  }
  /**/
}

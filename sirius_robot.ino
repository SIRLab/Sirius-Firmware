#include <Servo.h>

/*
  Sirius
  Robot designed to solve OBR objectives
  https://github.com/rafaelalmeidatk
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

//--------------------------------------------------
// IR Sensors

#define BLACK_VALUE             700
#define NUM_SENSORS             6

unsigned int sensorPins[NUM_SENSORS] = { A0, A1, A2, A3, A4, A5 };
unsigned int sensorValues[NUM_SENSORS];
long sensorLastPositionValue = 0;

//--------------------------------------------------
// Color sensors

#define USE_COLOR_SENSORS        true

#define RED_MINIMUM              60 // 60
#define GREEN_MINIMUM            45 // 45
#define BLUE_MINIMUM             40 //  40

const int CS_ESQ = 0;
const int CS_DIR = 1;
const int S0 = 0;
const int S1 = 1;
const int S2 = 2;
const int S3 = 3;
const int OUT = 4;

int colorSensors[2][5] = {
  {23, 25, 24, 27, 26},
  {51, 50, 49, 48, 47}
};

boolean gotGreen[2] = { false, false };
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

const int ULTRA_SR_TRIG_PIN = 42;
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

//--------------------------------------------------
// Rescue mode

bool rescueMode = false;
bool holdingBody = false;

//--------------------------------------------------
// System millis

unsigned long systemMillis = 0;

//--------------------------------------------------
// Calibration

#define CALIBRATION              2

//--------------------------------------------------

void moveRobot(int moveId) {
  servos[SV_LEFT].write(moves[SV_LEFT][moveId]);
  servos[SV_RIGHT].write(moves[SV_RIGHT][moveId]);
}

void clawMove(int cMove) {
  switch (cMove) {
    case OPEN:
      clawServos[CLAW].write(110);
      break;
    case CLOSE:
      clawServos[CLAW].write(0);
      break;
    case GO_UP:
      clawServos[HEIGHT].write(180);
      break;
    case GO_DOWN:
      clawServos[HEIGHT].write(60);
      break;
  }
}

void clawInit() {
  clawMove(OPEN);
  clawMove(GO_UP);
}

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

  return pulseWidth >= 10200 ? 0 : (pulseWidth / 29 / 2);
}

void readSensors(boolean d) {
  boolean debug = d || false;
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
    if (debug) {
      Serial.print(sensorValues[i]);
      Serial.print('\t');
    }
  }
  
  if (debug) {
    Serial.println();
    delay(200);
  }
  /**/
}

void readColorSensors(boolean d, int start, int maxIndex) {
  if (!USE_COLOR_SENSORS) {
    return;
  }
  boolean debug = d || false;
  for (int i = start; i < maxIndex; i++) {
    int red = 0;
    int green = 0;
    int blue = 0;

    int out = colorSensors[i][OUT];

    digitalWrite(colorSensors[i][S2], LOW);
    digitalWrite(colorSensors[i][S3], LOW);
    red = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);

    digitalWrite(colorSensors[i][S3], HIGH);
    blue = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);

    digitalWrite(colorSensors[i][S2], HIGH);
    green = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);

    int greenRedDifference = abs(green - red);
    int greenBlueDifference = abs(green - blue);

    if (greenRedDifference < 30 && greenBlueDifference < 20 && green > red && green > blue && red > RED_MINIMUM && green > GREEN_MINIMUM && blue > BLUE_MINIMUM) {
      Serial.print("GOT GREEN!"); // 98 - 64 - 84
      Serial.println(i);
      gotGreen[i] = true;
    } else {
      gotGreen[i] = false;
    }

    if (debug) {
      const char* sName = i == CS_ESQ ? "=== SENSOR ESQ ===" : "=== SENSOR DIR ===";
      //Serial.println(sName);
      Serial.print("R Intensity:");
      Serial.print(red, DEC);
      Serial.print(" G Intensity: ");
      Serial.print(green, DEC);
      Serial.print(" B Intensity : ");
      Serial.println(blue, DEC);
      delay(100);
    }
  }
  /**/
}

void readAllSensors() {
  readSensors(false);
  readColorSensors(false, 0, 2);
}

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

boolean isBlack(int sensorIndex) {
  return sensorValues[sensorIndex] > BLACK_VALUE;
}

boolean isCenterBlack() {
  int error = calculatePosition() - 2500;
  return error > -1000 && error < 1000;
}

boolean got90Right() {
  return isBlack(3) && isBlack(4) && isBlack(5);
}

boolean got90Left() {
  return isBlack(0) && isBlack(1) && isBlack(2);
}

boolean isAllBlack() {
  return isBlack(0) && isBlack(1) && isBlack(2) && isBlack(3) && isBlack(4) && isBlack(5);
}

boolean isAllWhite() {
  return !isBlack(0) && !isBlack(1) && !isBlack(2) && !isBlack(3) && !isBlack(4) && !isBlack(5);
}

void performObstacleEvade() {
  moveRobot(BACK);
  delay(500);
  moveRobot(RIGHT);
  delay(2100);
  moveRobot(FRONT);
  delay(1700);
  moveRobot(LEFT);
  delay(2400);
  moveRobot(FRONT);
  delay(4500);
  moveRobot(LEFT);
  delay(2100);
  moveRobot(FRONT);
  delay(2000);
  moveRobot(RIGHT);
  delay(1800);
  moveRobot(BACK);
  delay(2000);
}

void processRescueMode() {
  
}

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
    pinMode(colorSensors[i][S0], OUTPUT);
    pinMode(colorSensors[i][S1], OUTPUT);
    pinMode(colorSensors[i][S2], OUTPUT);
    pinMode(colorSensors[i][S3], OUTPUT);
    pinMode(colorSensors[i][OUT], INPUT);
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

void loop() {
  if (CALIBRATION == 1) {
    readSensors(true);
    return;
  } else if (CALIBRATION == 2) {
    readColorSensors(true, 0, 1);
    return;
  } else if (CALIBRATION == 3) {
    readColorSensors(true, 1, 2);
    return;
  }

  systemMillis = millis();

  if (rescueMode) {
    processRescueMode();
    return;
  }
  
  readAllSensors();

  if (digitalRead(OBSTC_BUTTON_PIN) == HIGH) {
    Serial.println("GOT OBSTACLE!");
    performObstacleEvade();
  }

  if ((isAllWhite() || isAllBlack()) && (!needTurnLeft && !needTurnRight)) {
    moveRobot(FRONT);
    return;
  } else if (!needTurnLeft && (got90Right() || needTurnRight)) {
    Serial.println("GOT 90 RIGHT!");
    boolean k = true;
    boolean k2 = false;
    moveRobot(FRONT);
    delay(100);
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
    while (k) {
      readAllSensors();
      if (isAllWhite() || needTurnRight) {
        Serial.println("Need to turn");
        k2 = true;
        if (needTurnRight) {
          moveRobot(RIGHT);
          delay(1200);
          moveRobot(FRONT);
          delay(100);
        } else {
          moveRobot(FRONT);
          delay(100);
          moveRobot(RIGHT);
          delay(500);
        }
        while (k2) {
          readSensors(false);
          if (isBlack(4)) {
            k2 = false;
            k = false;
            needTurnLeft = false;
            needTurnRight = false;
          }
        }
      } else {
        Serial.println("Continue to front");
      }
      k = false;
    }
    Serial.println("DONE 90 right!");
  } else if (!needTurnRight && (got90Left() || needTurnLeft)) {
    Serial.println("GOT 90 LEFT!");
    boolean k = true;
    boolean k2 = false;
    moveRobot(FRONT);
    delay(100);
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
    while (k) {
      readAllSensors();
      if (isAllWhite() || needTurnLeft) {
        Serial.println("Need to turn");
        k2 = true;
        if (needTurnLeft) {
          moveRobot(LEFT);
          delay(1200);
          moveRobot(FRONT);
          delay(100);
        } else {
          moveRobot(FRONT);
          delay(100);
          moveRobot(LEFT);
          delay(500);
        }
        while (k2) {
          readSensors(false);
          if (isBlack(1)) {
            k2 = false;
            k = false;
            needTurnLeft = false;
            needTurnRight = false;
          }
        }
      } else {
        Serial.println("Continue to front");
      }
      k = false;
    }
    Serial.println("DONE 90 left!");
  }

  int position = calculatePosition();
  int error = position - 2500;

  if (error < -700) {
    moveRobot(LEFT);
    if (error < -900) {
      delay(30);
    }
    if (error < -1000) {
      delay(40);
    }
    if (error < -1200) {
      delay(50);
    }
  } else if (error > 700) {
    moveRobot(RIGHT);
    if (error > 900) {
      delay(30);
    }
    if (error > 1000) {
      delay(40);
    }
    if (error > 1200) {
      delay(50);
    }
  } else {
    moveRobot(FRONT);
  }
  /**/
}

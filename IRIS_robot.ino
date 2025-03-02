//Servos
#define SERWO_A 12
#define SERWO_B 11

//Distance sensors
#define SHARP_FORWARD_LB 39
#define SHARP_FORWARD_MB 41
#define SHARP_RIGHT_LB 43
#define SHARP_RIGHT_MB 45
#define SHARP_LEFT_LB 47
#define SHARP_LEFT_MB 49
#define HCSR_FORWARD_LB 48
#define HCSR_FORWARD_MB 46
#define HCSR_RIGHT_LB 44
#define HCSR_RIGHT_MB 42
#define HCSR_LEFT_LB 40
#define HCSR_LEFT_MB 38

//Color sensors
#define C1 A13
#define C2 A12
#define C3 A14
#define C4 A3
#define C5 A0
#define C6 A2
#define C7 A4
#define C8 A1
#define C9 A7
#define C10 A15
#define C11 A6
#define C12 A8
#define C13 A10
#define C14 A9
#define C15 A5
#define C16 A11
#define THRESHOLD 400

//Stepper motors
#define R_STEP 51
#define L_STEP 50
#define R_DIR 53
#define L_DIR 52
#define MICROSTEP 16
#define Td 180

//Misc
#define BUZZER 8

//Location of our robot
struct coordinates {
  int posX;
  int posY;
  int rot;  // 0-forw, 1- right, 2-back, 3-left
};

coordinates loc;

unsigned long timePassed;

//Gripper things
void GripperUp() {
  //servo A 20
  // servo B 160

  for(int i = 0; i<200; i++){
    digitalWrite(SERWO_A, HIGH);
    digitalWrite(SERWO_B, HIGH);

    delayMicroseconds(750);
    digitalWrite(SERWO_A, LOW);
    delayMicroseconds(1400);
    digitalWrite(SERWO_B, LOW);
    delayMicroseconds(17850);
  }
}

void GripperDown() {
  for(int i = 0; i<200; i++){
    //a 105
    //b 75
    digitalWrite(SERWO_A, HIGH);
    digitalWrite(SERWO_B, HIGH);
    delayMicroseconds(1300);
    digitalWrite(SERWO_B, LOW);
    delayMicroseconds(300);
    digitalWrite(SERWO_A, LOW);
    delayMicroseconds(18400);
  }
}


//Read distance sensors
int readSharp1() {
  return digitalRead(SHARP_LEFT_MB) * 2 + digitalRead(SHARP_LEFT_LB) * 1;
}
int readSharp2() {
  return digitalRead(SHARP_FORWARD_MB) * 2 + digitalRead(SHARP_FORWARD_LB) * 1;
}
int readSharp3() {
  return digitalRead(SHARP_RIGHT_MB) * 2 + digitalRead(SHARP_RIGHT_LB) * 1;
}

int ultra1, ultra2, ultra3 = 0;
int readUltra1() {
  ultra1 = digitalRead(HCSR_LEFT_MB) * 2 + digitalRead(HCSR_LEFT_LB) * 1;
  return ultra1;
}
int readUltra2() {
  ultra2 = digitalRead(HCSR_FORWARD_MB) * 2 + digitalRead(HCSR_FORWARD_LB) * 1;
  return ultra2;
}
int readUltra3() {
  ultra3 = digitalRead(HCSR_RIGHT_MB) * 2 + digitalRead(HCSR_RIGHT_LB) * 1;
  return ultra3;
}

//Stepper motors functions
void rightMotorStep(int steps) {
  for (int x = 0; x < steps; x++) {
    digitalWrite(R_STEP, HIGH);
    delayMicroseconds(Td);
    digitalWrite(R_STEP, LOW);
    delayMicroseconds(Td);
  }
}
void leftMotorStep(int steps) {
  for (int x = 0; x < steps; x++) {
    digitalWrite(L_STEP, HIGH);
    delayMicroseconds(Td);
    digitalWrite(L_STEP, LOW);
    delayMicroseconds(Td);
  }
}
void bothMotorStep(int steps, int tdCoef = 1) {
  for (int x = 0; x < steps; x++) {
    digitalWrite(L_STEP, HIGH);
    digitalWrite(R_STEP, HIGH);
    delayMicroseconds(Td * tdCoef);
    digitalWrite(L_STEP, LOW);
    digitalWrite(R_STEP, LOW);
    delayMicroseconds(Td * tdCoef);
  }
}
void setDir(int dir = 0) {  //0 - forw, 1 - back
  if (dir == 0) {
    digitalWrite(R_DIR, LOW);
    digitalWrite(L_DIR, LOW);
  } else {
    digitalWrite(R_DIR, HIGH);
    digitalWrite(L_DIR, HIGH);
  }
  delay(10);
}
bool readSensor(int sensPin) {
  return (analogRead(sensPin) > THRESHOLD);
}

float wheelRotations;
int32_t stepCount;
void moveForw(int dist, int dir = 0) {
  setDir(dir);
  wheelRotations = (float)dist / 282.735;
  stepCount = int32_t(wheelRotations * 200 * MICROSTEP) / 12;

  if (dir == 0) {
    for (int i = 0; i < stepCount; i++) {
      if(i%45 == 0){
        while(readUltra2() > 1){} //stop until there is no opp
      }
      if (readSensor(C5)) {
        for (int j = 0; j < 4; j++) {
          bothMotorStep(2);
          rightMotorStep(1);
        }
      } else if (readSensor(C7)) {
        for (int j = 0; j < 4; j++) {
          bothMotorStep(2);
          leftMotorStep(1);
        }
      } else if (readSensor(C6)) {
        bothMotorStep(12);
      } else if (readSensor(C4)) {
        for (int j = 0; j < 6; j++) {
          bothMotorStep(1);
          rightMotorStep(1);
        }
      } else if (readSensor(C8)) {
        for (int j = 0; j < 6; j++) {
          bothMotorStep(1);
          leftMotorStep(1);
        }
      }
    }
  } else {
    for (int i = 0; i < stepCount; i++) {
      if (readSensor(C13)) {
        for (int j = 0; j < 4; j++) {
          bothMotorStep(2);
          leftMotorStep(1);
        }
      } else if (readSensor(C15)) {
        for (int j = 0; j < 4; j++) {
          bothMotorStep(2);
          rightMotorStep(1);
        }
      } else if (readSensor(C14)) {
        bothMotorStep(12);
      } else if (readSensor(C16)) {
        for (int j = 0; j < 6; j++) {
          bothMotorStep(1);
          leftMotorStep(1);
        }
      } else if (readSensor(C12)) {
        for (int j = 0; j < 6; j++) {
          bothMotorStep(1);
          rightMotorStep(1);
        }
      }
    }
  }
  delay(150);
}
void moveForwSensorless(int dist, int dir = 0) {
  setDir(dir);
  wheelRotations = (float)dist / 282.735;
  stepCount = int32_t(wheelRotations * 200 * MICROSTEP) / 12;

  for (int i = 0; i < stepCount; i++) {
    bothMotorStep(12);
  }
  delay(150);
}
void moveForwUntilSides(int tdCoef = 1, int dir = 0) {
  setDir(dir);
  while (!(readSensor(C2) || readSensor(C10))) {
    bothMotorStep(1);
  }
}

void rotateBy(int angle, bool sensored = true) {
  if (angle > 0) {
    digitalWrite(L_DIR, LOW);
    digitalWrite(R_DIR, HIGH);
  } else {
    digitalWrite(L_DIR, HIGH);
    digitalWrite(R_DIR, LOW);
  }
  delay(10);
  wheelRotations = (float)(abs(angle) / 360.0 * 2);
  stepCount = (int32_t)(wheelRotations * 200 * MICROSTEP);

  bothMotorStep((int)((float)stepCount * (3.0 / 4.0)));

  if (!f) {
    bothMotorStep((int)((float)stepCount * (1.0 / 4.0)));
  } else {
    switch (angle) {
      case -90:
        while (!readSensor(C3)) {
          bothMotorStep(1);
        }
        break;
      case 90:
        while (!readSensor(C9)) {
          bothMotorStep(1);
        }
        break;
      case 180:
        while (!readSensor(C14)) {
          bothMotorStep(1);
        }
        break;
    }
  }
  switch (angle) {
    case -90:
      if (loc.rot - 1 >= 0)
        updateLoc(loc.posX, loc.posY, loc.rot - 1);
      else
        updateLoc(loc.posX, loc.posY, 3);
      break;
    case 90:
      if (loc.rot + 1 <= 3)
        updateLoc(loc.posX, loc.posY, loc.rot + 1);
      else
        updateLoc(loc.posX, loc.posY, 0);
      break;
    case 180:
      if (loc.rot + 2 <= 3)
        updateLoc(loc.posX, loc.posY, loc.rot + 2);
      else
        updateLoc(loc.posX, loc.posY, loc.rot - 2);
      break;
  }
  delay(150);
}

void updateLoc(int x, int y, int rot) {
  loc.posX = x;
  loc.posY = y;
  loc.rot = rot;
}

bool moveTo(int x, int y) {
  if(readUltra2() > 1){
    if(loc.rot == 1)
      rotateBy(90, false);
    else if(loc.rot == 3)
      rotateBy(-90, false);
    beep(2);
    return true;
  }
  switch (loc.rot) {
    case 0:
      for (int i = 0; i < abs(y - loc.posY); i++) {
        moveForw(250, y > loc.posY ? 0 : 1);
        moveForwUntilSides(1, y > loc.posY ? 0 : 1);
      }
      if (x != loc.posX) {
        rotateBy(x > loc.posX ? 90 : -90, false);
        if(readUltra2() > 1){
          return true;
          break;
        }
      }
      for (int j = 0; j < abs(x - loc.posX); j++) {
        moveForw(250);
        moveForwUntilSides();
      }
      break;
    case 1:
      for (int i = 0; i < abs(x - loc.posX); i++) {
        moveForw(250, x > loc.posX ? 0 : 1);
        moveForwUntilSides(1, x > loc.posX ? 0 : 1);
      }
      if (y != loc.posY) {
        rotateBy(y > loc.posY ? -90 : 90, false);
        if(readUltra2() > 1){
          return true;
          break;
        }
      }
      for (int j = 0; j < abs(y - loc.posY); j++) {
        moveForw(250);
        moveForwUntilSides();
      }
      break;
    case 2:
      for (int i = 0; i < abs(y - loc.posY); i++) {
        moveForw(250, y > loc.posY ? 1 : 0);
        moveForwUntilSides(1, y > loc.posY ? 1 : 0);
      }
      if (x != loc.posX) {
        rotateBy(x > loc.posX ? -90 : 90, false);
        if(readUltra2() > 1){
          return true;
          break;
        }
      }
      for (int j = 0; j < abs(x - loc.posX); j++) {
        moveForw(250);
        moveForwUntilSides();
      }
      break;
    case 3:
      for (int i = 0; i < abs(x - loc.posX); i++) {
        moveForw(250, x > loc.posX ? 1 : 0);
        moveForwUntilSides(1, x > loc.posX ? 1 : 0);
      }
      if (y != loc.posY) {
        rotateBy(y > loc.posY ? 90 : -90, false);
        if(readUltra2() > 1){
          return true;
          break;
        }
      }
      for (int j = 0; j < abs(y - loc.posY); j++) {
        moveForw(230);
        moveForwUntilSides();
      }
      break;
  }
  updateLoc(x, y, loc.rot);
  beep(1);
  return false;
}

void moveForward() {
  switch (loc.rot) {
    case 0:
      moveTo(loc.posX, loc.posY + 1);
      break;
    case 1:
      moveTo(loc.posX + 1, loc.posY);
      break;
    case 2:
      moveTo(loc.posX, loc.posY - 1);
      break;
    case 3:
      moveTo(loc.posX - 1, loc.posY);
      break;
  }
}


int canCount = 0;
void goToBase() {
  moveTo(2, 1);
  moveTo(2, 0);
  if (canCount % 2 == 0) {
    if (loc.rot == 0) {
      rotateBy(90, false);
    } else if (loc.rot == 2) {
      rotateBy(-90, false);
    } else if (loc.rot == 3) {
      rotateBy(180, false);
    }
  } else {
    if (loc.rot == 0) {
      rotateBy(-90, false);
    } else if (loc.rot == 2) {
      rotateBy(90, false);
    } else if (loc.rot == 3) {
      rotateBy(180, false);
    }
  }
  //Move only to bring up gripper
  moveForw(150);
  GripperUp();

  //Push the can to the maximum
  moveForw(380 - (canCount / 2) * 60);
  moveForw(380 - (canCount / 2) * 60, 1);

  moveForwUntilSides(1, 1);
  if (canCount % 2 == 0)
    rotateBy(-90, false);
  else
    rotateBy(90, false);
  canCount++;
}

int sharp1, sharp2, sharp3;
bool checkForCans() {

  delay(400);

  sharp1 = readSharp1();
  sharp2 = readSharp2();
  sharp3 = readSharp3();

  delay(100);

  //sharp1 += readSharp1();
  sharp2 += readSharp2();
  sharp3 += readSharp3();

  delay(100);

  //sharp1 += readSharp1();
  //sharp2 += readSharp2();
  sharp3 += readSharp3();

  //sharp1 /= 3;
  sharp2 /= 2;
  sharp3 /= 3;

  if(millis() - timePassed > 180000){
    beep(5);
    while(1);
  }


  if (sharp2 >= 2 && readUltra2() <= 2) {
    beep(3);
    moveForw(250);
    GripperDown();
    moveForwUntilSides(1, 1);
    goToBase();
    return true;
  } else if (sharp3 >= 2) {
    beep(4);
    rotateBy(90, false);
    moveForw(250);
    GripperDown();
    moveForwUntilSides(1, 1);
    goToBase();
    return true;
  } else if (sharp1 >= 2) {
    beep(5);
    rotateBy(-90, false);
    moveForw(250);
    GripperDown();
    moveForwUntilSides(1, 1);
    goToBase();
    return true;
  }
  return false;
}

void printLoc() {
  Serial.println("X: " + String(loc.posX) + "; Y: " + String(loc.posY) + "; Rot: " + String(loc.rot));
}

void beep(int x){
  for(int i = 0; i < x; i++){
    digitalWrite(BUZZER, HIGH);
    delay(50);
    digitalWrite(BUZZER, LOW);
    delay(50);
  }
}

void setup() {

  //Setting pins
  pinMode(SHARP_FORWARD_LB, INPUT);
  pinMode(SHARP_FORWARD_MB, INPUT);
  pinMode(SHARP_RIGHT_LB, INPUT);
  pinMode(SHARP_RIGHT_MB, INPUT);
  pinMode(SHARP_LEFT_LB, INPUT);
  pinMode(SHARP_LEFT_MB, INPUT);
  pinMode(HCSR_FORWARD_LB, INPUT);
  pinMode(HCSR_FORWARD_MB, INPUT);
  pinMode(HCSR_RIGHT_LB, INPUT);
  pinMode(HCSR_RIGHT_MB, INPUT);
  pinMode(HCSR_LEFT_LB, INPUT);
  pinMode(HCSR_LEFT_MB, INPUT);
  pinMode(L_DIR, OUTPUT);
  pinMode(R_DIR, OUTPUT);
  pinMode(L_STEP, OUTPUT);
  pinMode(R_STEP, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(SERWO_A, OUTPUT);
  pinMode(SERWO_B, OUTPUT);

  Serial.begin(1200);
  GripperUp();

  delay(1000);
  timePassed = millis();
  moveForwUntilSides();
  updateLoc(2, 0, 0);
}

void loop(){
  if (moveTo(2, 1)) return;
  if (checkForCans()) return;
  if (moveTo(2, 2)) return;
  if (checkForCans()) return;
  if (moveTo(1, 2)) return;
  if (checkForCans()) return;
  if (moveTo(0, 2)) return;
  rotateBy(180, false);
  if (checkForCans()) return;
  if (moveTo(1, 2)) return;
  if (checkForCans()) return;
  if (moveTo(2, 2)) return;
  if (checkForCans()) return;
  if (moveTo(3, 2)) return;
  if (checkForCans()) return;
  if (moveTo(4, 2)) return;
  if (checkForCans()) return;
  rotateBy(180, false);
  if (checkForCans()) return;
  if (moveTo(3, 2)) return;
  if (checkForCans()) return;
  if (moveTo(2, 2)) return;
}
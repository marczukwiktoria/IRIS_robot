#include <Arduino_FreeRTOS.h>
#include <queue.h> // queue something

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
#define THRESHOLD 600 //Reflective sensor indicates value of around 900 when on black


// Arrays storing pin numbers for line sensors
// int frontLineSensorPins[] = {C4, C5, C6, C7, C8}; //Left to right 
// int backLineSensorPins[] = {C16, C15, C14, C13, C12}; // Left to right
int leftLineSensorPins[] = {C3, C2, C1};
int rightLineSensorPins[] = {C9, C10, C11};

//Stepper motors
#define R_STEP 51
#define L_STEP 50
#define R_DIR 53
#define L_DIR 52
#define MICROSTEP 16
#define Td 150

// Function to read sensor value and return a boolean state
bool readSensor(int sensPin){
  return (analogRead(sensPin) > THRESHOLD);
}

// Function prototypes for FreeRTOS tasks
void FlagStep(void *pvParameters);
void DisplayToSerial(void *pvParameters);
void MakeMeasurements(void *pvParameters);

// Other functions used by a robot 
int readSharp1();
int readSharp2();
int readSharp3();
int readUltra1();
int readUltra2();
int readUltra3();
void rightMotorStep(int steps);
void leftMotorStep(int steps);
void bothMotorStep(int steps);
void moveStraight(int dist, int dir = 0);
void rotateBy(int angle, bool sensored = true);
void returnCans();
void calculatePath();
void makeDecision(int x, int y);
void gripper(int direction); // 0 -> Up, 1-> Down

// Motor control flags
int remainingCrossings = 4; //Crossings remaining, 0 - stop at the next crossing
bool keepLineOn = false; //flag for detecting crossing
unsigned int previousCrossingTimestamp = millis(); // Timer for detecting crossing in case of crossing the crossing not straight 
int cansCount = 0;
int stepsDone = 0;

// Motor action settings
enum RobotAction {Straighten=0, RotateLeft=-90, RotateRight=90, Retreat =-1};
volatile RobotAction CurrentAction = Straighten;
// Structure to store robot position and orientation
struct coordinates {
  int posX = 2;
  int posY = 0;
  int rot = 0;  // 0-forw, 90-right, 180-back, -90-left
};
coordinates robotPosition;
bool isRotating = false;

// Sensor variables
QueueHandle_t makeMeasurementsQueue;

// Going squares
int n = 3;
int n_coeff = 1;
bool change_n = false;
bool opponentAhead = false;
int rectanglePoints[4][2] = {{1,4},{1,1},{3,1},{3,4}};
// int rectanglePoints[4][2] = {{1,2},{1,1},{3,1},{3,2}};
int cantrix[5][5] = {{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0}};
int distanceCost[5][5] = {{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0}};

// Variables used for returning to base
bool goingToBase = false;
bool puttingBackCans = false;

//Misc
#define BUZZER 8

// Functions to read distance sensor values
int readSharp1() {
  return digitalRead(SHARP_LEFT_MB) * 2 + digitalRead(SHARP_LEFT_LB) * 1;
}
int readSharp2() {
  return digitalRead(SHARP_FORWARD_MB) * 2 + digitalRead(SHARP_FORWARD_LB) * 1;
}
int readSharp3() {
  return digitalRead(SHARP_RIGHT_MB) * 2 + digitalRead(SHARP_RIGHT_LB) * 1;
}

// Ultrasonic sensor readings
int readUltra1() {
  return digitalRead(HCSR_LEFT_MB) * 2 + digitalRead(HCSR_LEFT_LB) * 1;
}
int readUltra2() {
  return digitalRead(HCSR_FORWARD_MB) * 2 + digitalRead(HCSR_FORWARD_LB) * 1;
}
int readUltra3() {
  return digitalRead(HCSR_RIGHT_MB) * 2 + digitalRead(HCSR_RIGHT_LB) * 1;
}

//Right stepper motor function
void rightMotorStep(int steps){
  for(int x = 0; x < steps; x++){
    digitalWrite(R_STEP, HIGH);
    delayMicroseconds(Td);
    digitalWrite(R_STEP, LOW);
    delayMicroseconds(Td);
  }
}

//Left stepper motor function
void leftMotorStep(int steps){
  for(int x = 0; x < steps; x++){
    digitalWrite(L_STEP, HIGH);
    delayMicroseconds(Td);
    digitalWrite(L_STEP, LOW);
    delayMicroseconds(Td);
  }
}

//Both stepper motors function
void bothMotorStep(int steps){
  for(int x = 0; x < steps; x++){
    digitalWrite(L_STEP, HIGH);
    digitalWrite(R_STEP, HIGH);
    delayMicroseconds(Td);
    digitalWrite(L_STEP, LOW);
    digitalWrite(R_STEP, LOW);
    delayMicroseconds(Td);
  }
}

//Function used for straight movement
void moveStraight(int dist, int dir = 0){
  digitalWrite(R_DIR, LOW);
  digitalWrite(L_DIR, LOW);
  float wheelRotations = (float)dist / 282.735;
  int32_t stepCount = int32_t(wheelRotations * 200 * MICROSTEP) / 12;
  if(dir == 0){
    for(int i = 0; i < stepCount; i++){
      if(readSensor(C5)){
        for(int j = 0; j < 4; j++){
          bothMotorStep(2);
          rightMotorStep(1);
        }
      } else if(readSensor(C7)){
        for(int j = 0; j < 4; j++){
          bothMotorStep(2);
          leftMotorStep(1);
        }
      } else if(readSensor(C6)){
        bothMotorStep(12);
      } else if(readSensor(C4)){
        for(int j = 0; j < 6; j++){
          bothMotorStep(1);
          rightMotorStep(1);
        }
      } else if(readSensor(C8)){
        for(int j = 0; j < 6; j++){
          bothMotorStep(1);
          leftMotorStep(1);
        }
      } else {
        bothMotorStep(3);
      }

      //Below is intersection detection
      unsigned int tempTime = abs(millis() - previousCrossingTimestamp);
      if ((readSensor(rightLineSensorPins[2]) || readSensor(leftLineSensorPins[2])) && keepLineOn == false ) {
        keepLineOn = true;
        previousCrossingTimestamp = millis();
        if (robotPosition.rot == 0){
          robotPosition.posY = robotPosition.posY + 1;
        } else if (robotPosition.rot == 180) {
          robotPosition.posY = robotPosition.posY - 1;
        } else if (robotPosition.rot == 90) {
          robotPosition.posX = robotPosition.posX + 1;
        } else if (robotPosition.rot == -90) {
          robotPosition.posX = robotPosition.posX - 1;
        }
        stepsDone=0;
        int one = 1;
        xQueueSend(makeMeasurementsQueue,&one,portMAX_DELAY);
      } else if (!readSensor(rightLineSensorPins[2]) && !readSensor(leftLineSensorPins[2]) && tempTime>=800) {
        keepLineOn = false;
      }
    }
  } else {
    digitalWrite(R_DIR, HIGH);
    digitalWrite(L_DIR, HIGH);
    for(int i = 0; i < stepCount; i++){
      if(readSensor(C5)){
        for(int j = 0; j < 4; j++){
          bothMotorStep(2);
          leftMotorStep(1);
        }
      } else if(readSensor(C7)){
        for(int j = 0; j < 4; j++){
          bothMotorStep(2);
          rightMotorStep(1);
        }
      } else if(readSensor(C6)){
        bothMotorStep(12);
      } else if(readSensor(C4)){
        for(int j = 0; j < 6; j++){
          bothMotorStep(1);
          leftMotorStep(1);
        }
      } else if(readSensor(C8)){
        for(int j = 0; j < 6; j++){
          bothMotorStep(1);
          rightMotorStep(1);
        }
      }
      //No logic for driving backwards, just go back
      unsigned int tempTime = abs(millis() - previousCrossingTimestamp);
      if ((readSensor(rightLineSensorPins[2]) || readSensor(leftLineSensorPins[2])) && keepLineOn == false ) {
        previousCrossingTimestamp = millis();
        keepLineOn = true;
        stepsDone = 0;
        int one = 1;
        xQueueSend(makeMeasurementsQueue,&one,portMAX_DELAY);
      } else if (!readSensor(rightLineSensorPins[2]) && !readSensor(leftLineSensorPins[2]) && tempTime>=400) {
        keepLineOn = false;
      }
    }
  }
}

void rotateBy(int angle, bool sensored = true) {
  // Serial.println("PRE-ROTATE"); 
  // Serial.flush();
  // if (cansCount != 0 ) {gripper(1);}
  if (angle > 0){
    digitalWrite(L_DIR, LOW);
    digitalWrite(R_DIR, HIGH);
  } else {
    digitalWrite(L_DIR, HIGH);
    digitalWrite(R_DIR, LOW);
  }

  float wheelRotations = (float)(abs(angle) / 360.0 * 2);
  int32_t stepCount = (int32_t)(wheelRotations * 200 * MICROSTEP);

  bothMotorStep((int)((float)stepCount*(3.0/4.0)));

  if(!sensored){
    bothMotorStep((int)((float)stepCount*(1.0/4.0)));
  }else{
    switch(angle){
      case -90:
        while(!readSensor(C3)){
          bothMotorStep(1);
        }
        break;
      case 90:
        while(!readSensor(C9)){
          bothMotorStep(1);
        }
        break;
      case 180:
        while(!readSensor(C14)){
          bothMotorStep(1);
        }
    }
  }
  if (angle == 90 && robotPosition.rot != 180){
    robotPosition.rot = robotPosition.rot + angle;
  } else if (angle == 90 && robotPosition.rot == 180){
    robotPosition.rot = -90;
  } else if (angle == -90 && robotPosition.rot != -90){
    robotPosition.rot = robotPosition.rot + angle;
  } else if (angle == -90 && robotPosition.rot == -90){
    robotPosition.rot = 180;
  } else if (angle == 180 && robotPosition.rot == 90){
    robotPosition.rot = -90;
  } else if (angle == 180 && robotPosition.rot == 180){
    robotPosition.rot = 0;
  } else if (angle == 180 && (robotPosition.rot == 0 || robotPosition.rot ==-90)){
    robotPosition.rot = robotPosition.rot + angle;
  }

  keepLineOn=true;
  previousCrossingTimestamp = millis();
  CurrentAction=Straighten;
  
  isRotating = false;
  // Serial.println("POST-ROTATE");
  // Serial.flush(); 
  int one = 1;
  xQueueSend(makeMeasurementsQueue,&one,portMAX_DELAY);
  // if (cansCount!= 0) {gripper(0);}
}

//Logic for placing cans in base
void returnCans() {
  if (robotPosition.posX == 1 && robotPosition.rot == 180) {
    rotateBy(90);
    moveStraight(120);
    gripper(0);
    moveStraight(80, 1);
    rotateBy(90);
  } else if (robotPosition.posX == 3 && robotPosition.rot == 180) {
    rotateBy(-90);
    moveStraight(120);
    gripper(0);
    moveStraight(80, 1);
    rotateBy(-90);
  } else { //we got to the x=2, y=0
    rotateBy(-90);
    moveStraight(380);
    moveStraight(120);
    gripper(0);
    moveStraight(120,1);
    rotateBy(-90);
  }

  goingToBase = false;
  puttingBackCans = false;
  cansCount = 0;
}

void OpponentDetection(void *pvParameters) {
  for (;;) {
    int frontSensorReading = readUltra2();
    if (frontSensorReading>=2 && CurrentAction==Straighten) {
      opponentAhead=true;
      if (stepsDone==0) {
        //jestesmy na skrzyzowaniu
      } else if (stepsDone>=211) { //211 is prime
        CurrentAction = Straighten;
      } else {
        CurrentAction = Retreat;
        bool canAtBoard = false;
        for (int i = 0;i<5;i++) {
          for (int j = 0;j<5;j++) {
            if (cantrix[i][j] == 1) {
              canAtBoard = true;
              break;
            }
          }
          if (canAtBoard == true) {break;} 
        }

        // There is no can on board, change the n counting direction
        if (canAtBoard == false) {
          n_coeff = -n_coeff;
          change_n = true;
        }
      }
    }
    vTaskDelay(200/portTICK_PERIOD_MS);
  }
}

// Function to adjust motor speed based on line tracking sensors
void FlagStep(void *pvParameters)
{
  for (;;){
    if (goingToBase==true && robotPosition.posY == 0 && puttingBackCans == false) {
      puttingBackCans = true;
      returnCans();
    } else if (CurrentAction == Straighten && puttingBackCans == false) {
      moveStraight(10);
      stepsDone+=10;
    } else if (CurrentAction == RotateRight && isRotating==false && puttingBackCans==false) {
      isRotating=true;
      rotateBy(90);
    } else if (CurrentAction == RotateLeft && isRotating == false && puttingBackCans == false) {
      isRotating=true;
      rotateBy(-90);
    } else if (CurrentAction == Retreat) {
      moveStraight(10, 1);
    }
  
    vTaskDelay(10/portTICK_PERIOD_MS);
  }
}

void MakeMeasurements(void *pvParameters) {
  int valueFromQueue = 0;
  for (;;) {
    if (xQueueReceive(makeMeasurementsQueue,&valueFromQueue,portMAX_DELAY) == pdPASS ) {
      if (valueFromQueue==1) {
        if (cantrix[robotPosition.posY][robotPosition.posX] == 1 && robotPosition.posY !=0) {
          Serial.println("we are at can position!");
          cantrix[robotPosition.posY][robotPosition.posX] = 0;
          cansCount+=1;
        }
        if (cantrix[robotPosition.posY][robotPosition.posX] == 1 && robotPosition.posY ==0) {
          Serial.println("we are at base position!");
          cantrix[robotPosition.posY][robotPosition.posX] = 0;

        }
        if (cantrix[robotPosition.posY][robotPosition.posX] == 2) {
          Serial.println("virtual can position!");
          cantrix[robotPosition.posY][robotPosition.posX] = 0; 
        }
        Serial.println("making measurements");
        int Sharp1 = readSharp1();
        int Sharp2 = readSharp2();
        int Sharp3 = readSharp3();
        moveStraight(1);
        if (Sharp1<2) {Sharp1 = readSharp1();}
        if (Sharp2<2) {Sharp2 = readSharp2();}
        if (Sharp3<2) {Sharp3 = readSharp3();}
        moveStraight(2);
        if (Sharp1<2) {Sharp1 = readSharp1();}
        if (Sharp2<2) {Sharp2 = readSharp2();}
        if (Sharp3<2) {Sharp3 = readSharp3();}

        int Ultra1 = readUltra1();
        int Ultra2 = readUltra2();
        int Ultra3 = readUltra3();
        if (Ultra2>=2) {opponentAhead = true;}
        //if ultra detects, it's enemy!
        Serial.print("ultras; ");
        Serial.print(Ultra1);
        Serial.print(";");
        Serial.print(Ultra2);
        Serial.print(";");
        Serial.print(Ultra3);
        Serial.println(";");

        if (
          !(robotPosition.posX == 0 && robotPosition.rot == 0) &&
          !(robotPosition.posX == 4 && robotPosition.rot == 180) &&
          !(robotPosition.posY <= 1 && robotPosition.rot == -90) &&
          !(robotPosition.posY == 4 && robotPosition.rot == 90) &&
          Ultra1 <= 1
        ) {
          if (Sharp1 >= 2 && robotPosition.rot == 0) {
            cantrix[robotPosition.posY][robotPosition.posX-1] = 1;
          } else if (Sharp1 >= 2 && robotPosition.rot == 180) {
            cantrix[robotPosition.posY][robotPosition.posX+1] = 1;
          } else if (Sharp1 >= 2 && robotPosition.rot == 90) {
            cantrix[robotPosition.posY+1][robotPosition.posX] = 1;
          } else if (Sharp1 >= 2 && robotPosition.rot == -90) {
            cantrix[robotPosition.posY-1][robotPosition.posX] = 1;
          }
        } else if (Ultra1>=2) {
          if (robotPosition.rot == 0) {
            for (int i = 0;i<robotPosition.posX;i++) {
              cantrix[robotPosition.posY][i] = 0;
            }
          } else if (robotPosition.rot == 180) {
            for (int i = robotPosition.posX;i<5;i++) {
              cantrix[robotPosition.posY][i] = 0;
            }
          } else if (robotPosition.rot == 90) {
            for (int i = robotPosition.posY;i<5;i++) {
              cantrix[i][robotPosition.posX] = 0;
            }
          } else if (robotPosition.rot == -90) {
            for (int i = 0;i<robotPosition.posY;i++) {
              cantrix[i][robotPosition.posX] = 0;
            }
          }
        }

        if (
          !(robotPosition.posX == 0 && robotPosition.rot == 180) &&
          !(robotPosition.posX == 4 && robotPosition.rot == 0) &&
          !(robotPosition.posY <= 1 && robotPosition.rot == 90) &&
          !(robotPosition.posY == 4 && robotPosition.rot == -90) &&
          Ultra3 <= 1
        ) {
          if (Sharp3 >= 2 && robotPosition.rot == 0){
            cantrix[robotPosition.posY][robotPosition.posX+1] = 1;
          } else if (Sharp3 >= 2 && robotPosition.rot == 180){
            cantrix[robotPosition.posY][robotPosition.posX-1] = 1;
          } else if (Sharp3 >= 2 && robotPosition.rot == 90){
            cantrix[robotPosition.posY-1][robotPosition.posX] = 1;
          } else if (Sharp3 >= 2 && robotPosition.rot == -90){
            cantrix[robotPosition.posY+1][robotPosition.posX] = 1;
          }
        } else if (Ultra3>=2) {
          if (robotPosition.rot == 0) {
            for (int i = robotPosition.posX;i<5;i++) {
              cantrix[robotPosition.posY][i] = 0;
            }
          } else if (robotPosition.rot == 180) {
            for (int i = 0;i<robotPosition.posX;i++) {
              cantrix[robotPosition.posY][i] = 0;
            }
          } else if (robotPosition.rot == 90) {
            for (int i = 0;i<robotPosition.posY;i++) {
              cantrix[i][robotPosition.posX] = 0;
            }
          } else if (robotPosition.rot == -90) {
            for (int i = robotPosition.posY;i<5;i++) {
              cantrix[i][robotPosition.posX] = 0;
            }
          }
        }

        if (cansCount==0) {
          if (
            !(robotPosition.posX == 0 && robotPosition.rot == -90) &&
            !(robotPosition.posX == 4 && robotPosition.rot == 90) &&
            !(robotPosition.posY <= 1 && robotPosition.rot == 180) &&
            !(robotPosition.posY == 4 && robotPosition.rot == 0) &&
            Ultra2 <= 1
          ) {
            if (Sharp2 >= 2 && robotPosition.rot == 0){
              cantrix[robotPosition.posY+1][robotPosition.posX] = 1;
            } else if (Sharp2 >= 2 && robotPosition.rot == 180){
              cantrix[robotPosition.posY-1][robotPosition.posX] = 1;
            } else if (Sharp2 >= 2 && robotPosition.rot == 90){
              cantrix[robotPosition.posY][robotPosition.posX+1] = 1;
            } else if (Sharp2 >= 2 && robotPosition.rot == -90){
              cantrix[robotPosition.posY][robotPosition.posX-1] = 1;
            }
          } else if (Ultra2>=2) {
          if (robotPosition.rot == 0) {
            for (int i = robotPosition.posX;i<5;i++) {
              cantrix[i][robotPosition.posX] = 0;
            }
          } else if (robotPosition.rot == 180) {
            for (int i = 0;i<robotPosition.posX;i++) {
              cantrix[i][robotPosition.posX] = 0;
            }
          } else if (robotPosition.rot == 90) {
            for (int i = robotPosition.posY;i<5;i++) {
              cantrix[robotPosition.posY][i] = 0;
            }
          } else if (robotPosition.rot == -90) {
            for (int i = 0;i<robotPosition.posY;i++) {
              cantrix[robotPosition.posY][i] = 0;
            }
          }
        }
        }
        calculatePath();
      }
    }
  }
}

void calculatePath() {
  Serial.println("calculating path");
  int y_lowest, x_lowest, lowestCost = 9999;
  bool canOnBoard = false;
  bool virtualCanOnBoard = false;
  for (int y = 1; y<5; y++) {
    for (int x = 0; x<5; x++) {
      if (cantrix[y][x] == 1) {
        canOnBoard = true;
        int distance = abs(robotPosition.posX - x) + abs(robotPosition.posY - y);
        if (robotPosition.rot == 0){
          if (x != robotPosition.posX){
            distance += 1;
          }
          if (y < robotPosition.posY){
            distance += 1;
          }
          if (x == robotPosition.posX && y < robotPosition.posY){
            distance += 1;
          }
        } else if (robotPosition.rot == 90){
          if (y != robotPosition.posY){
            distance += 1;
          }
          if (x < robotPosition.posX){
            distance += 1;
          }
          if (y == robotPosition.posY && x < robotPosition.posX){
            distance += 1;
          }

        } else if (robotPosition.rot == -90){
          if (y != robotPosition.posY){
            distance += 1;
          }
          if (x > robotPosition.posX){
            distance += 1;
          }
          if (y == robotPosition.posY && x > robotPosition.posX){
            distance += 1;
          }

        } else {
          if (x != robotPosition.posX){
            distance += 1;
          }
          if (y > robotPosition.posY){
            distance += 1;
          }
          if (x == robotPosition.posX && y > robotPosition.posY){
            distance += 1;
          }

        }
        distanceCost[y][x] = distance;
        //Can with lowest cost detection
        if (distanceCost[y][x] < lowestCost && distanceCost[y][x] != 0) {
          x_lowest = x;
          y_lowest = y;
          lowestCost = distanceCost[y][x];
        }
      } else if (cantrix[y][x]==2) {
        virtualCanOnBoard = true;
        cantrix[y][x] = 2;
        distanceCost[y][x] = -1;
      } else {
        cantrix[y][x] = 0;
        distanceCost[y][x] = 0;
      }
    }
  }
  
  if (canOnBoard==true) {
    int idx;
    if (n_coeff == 1) {
      idx = ((n-1)+4)%4;
    } else {
      idx = (n+1)%4;
    }
    if (cantrix[rectanglePoints[idx][1]][rectanglePoints[idx][0]] == 2) {
      cantrix[rectanglePoints[idx][1]][rectanglePoints[idx][0]] = 0;
    }
  }

  Serial.print("cc");
  Serial.print(cansCount);
  Serial.print(";");
  Serial.print(canOnBoard);
  Serial.print(";");
  Serial.println(virtualCanOnBoard);
  // Creating search path with virtual cans 
  if (canOnBoard == false && virtualCanOnBoard == false && cansCount == 0) {
    change_n=true;
  } 

  if (change_n == true) {
    cantrix[rectanglePoints[n][1]][rectanglePoints[n][0]] = 0;
    distanceCost[rectanglePoints[n][1]][rectanglePoints[n][0]] = 0;
    n = n + (1*n_coeff);
    if(n>3) {n=0;}
    if(n<0) {n=3;}
    cantrix[rectanglePoints[n][1]][rectanglePoints[n][0]] = 2;
    distanceCost[rectanglePoints[n][1]][rectanglePoints[n][0]] = -1;
    x_lowest = rectanglePoints[n][0];
    y_lowest = rectanglePoints[n][1];
    change_n = false;
  }

  // Go to base
  goingToBase = false;
  if (cansCount>=2 || (canOnBoard == false && cansCount == 1)) {
    distanceCost[0][robotPosition.posX] = -1;
    cantrix[0][robotPosition.posX] = 1;
    if (robotPosition.posX<2) {
      x_lowest = 1;
    } else {
      x_lowest = 3;
    }
    y_lowest = 0;
    goingToBase = true;
  }

  Serial.print("Target: X=");
  Serial.print(x_lowest);
  Serial.print(" Y=");
  Serial.print(y_lowest);
  Serial.print(" Lowest cost: ");
  Serial.println(lowestCost);
  makeDecision(x_lowest, y_lowest);
}  

// Function for decision making
void makeDecision(int x, int y) {
  Serial.println("making decision");
  // Calculate dist in x and y
  int y_dist = y - robotPosition.posY;
  int x_dist = x - robotPosition.posX;
  
  if (opponentAhead==false) {

    if (y == 0 && (x==1 || x==3) && x_dist !=0 && robotPosition.posX!=2) {
      if (robotPosition.posX==0) {
        if (robotPosition.rot == 0) {
          CurrentAction = RotateRight; // Obrót o 180°
        } else if (robotPosition.rot == 180) {
          CurrentAction = RotateLeft;
        } else if (robotPosition.rot == 90) {
          CurrentAction = Straighten;
          remainingCrossings = 0;
        } else if (robotPosition.rot == -90) { 
          CurrentAction = RotateLeft;
        }
      } else if (robotPosition.posX==4) {
        if (robotPosition.rot == 0) {
          CurrentAction = RotateLeft; // Obrót o 180°
        } else if (robotPosition.rot == 180) {
          CurrentAction = RotateRight;
        } else if (robotPosition.rot == -90) {
          CurrentAction = Straighten;
          remainingCrossings = 0;
        } else if (robotPosition.rot == 90) { 
          CurrentAction = RotateRight;
        }
      }
      return;
    }

    // Najpierw spróbuj poruszać się w aktualnym kierunku
    if (robotPosition.rot == 0 && y_dist > 0) { // Obrót 0° - ruch w górę (Y+)
      remainingCrossings = y_dist-1;
      CurrentAction = Straighten;
      return;
    }
    else if (robotPosition.rot == 180 && y_dist < 0) { // Obrót 180° - ruch w dół (Y-)
      remainingCrossings = -y_dist-1;
      CurrentAction = Straighten;
      return;
    }
    else if (robotPosition.rot == 90 && x_dist > 0) { // Obrót 90° - ruch w prawo (X+)
      remainingCrossings = x_dist-1;
      CurrentAction = Straighten;
      return;
    }
    else if (robotPosition.rot == -90 && x_dist < 0) { // Obrót -90° - ruch w lewo (X-)
      remainingCrossings = -x_dist-1;
      CurrentAction = Straighten;
      return;
    }

    // Jeśli nie możemy jechać prosto w aktualnym kierunku, wykonaj obrót
    if (abs(y_dist) >= abs(x_dist)) { // Priorytet dla osi Y jeśli odległość jest większa lub równa
      if (y_dist > 0) { // Potrzebny ruch w górę (Y+)
        if (robotPosition.rot == 180) {
          CurrentAction = RotateLeft; // Obrót o 180°
        }
        else if (robotPosition.rot == 90) {
          CurrentAction = RotateLeft; // Obrót w lewo (90° -> 0°)
        }
        else if (robotPosition.rot == -90) {
          CurrentAction = RotateRight; // Obrót w prawo (-90° -> 0°)
        }
      }
      else if (y_dist < 0) { // Potrzebny ruch w dół (Y-)
        if (robotPosition.rot == 0) {
          CurrentAction = RotateLeft; // Obrót o 180°
        }
        else if (robotPosition.rot == 90) {
          CurrentAction = RotateRight; // Obrót w prawo (90° -> 180°)
        }
        else if (robotPosition.rot == -90) {
          CurrentAction = RotateLeft; // Obrót w lewo (-90° -> 180°)
        }
      }
    } else { // Priorytet dla osi X
      if (x_dist > 0) { // Potrzebny ruch w prawo (X+)
        if (robotPosition.rot == 0) {
          CurrentAction = RotateRight; // Obrót w prawo (0° -> 90°)
        }
        else if (robotPosition.rot == 180) {
          CurrentAction = RotateLeft; // Obrót w lewo (180° -> 90°)
        }
        else if (robotPosition.rot == -90) {
          CurrentAction = RotateLeft; // Obrót o 180°
        }
      }
      else if (x_dist < 0) { // Potrzebny ruch w lewo (X-)
        if (robotPosition.rot == 0) {
          CurrentAction = RotateLeft; // Obrót w lewo (0° -> -90°)
        }
        else if (robotPosition.rot == 180) {
          CurrentAction = RotateRight; // Obrót w prawo (180° -> -90°)
        }
        else if (robotPosition.rot == 90) {
          CurrentAction = RotateLeft; // Obrót o 180°
        }
      }
    } 
  } else { // HERE IS A LOGIC WHEN OPPONENT IS AHEAD
    if (abs(y_dist) < abs(x_dist)) { // Priorytet dla osi Y jeśli odległość jest większa lub równa
      if (y_dist > 0) { // Potrzebny ruch w górę (Y+)
        if (robotPosition.rot == 180) {
          CurrentAction = RotateLeft; // Obrót o 180°
        }
        else if (robotPosition.rot == 90) {
          CurrentAction = RotateLeft; // Obrót w lewo (90° -> 0°)
        }
        else if (robotPosition.rot == -90) {
          CurrentAction = RotateRight; // Obrót w prawo (-90° -> 0°)
        }
      }
      else if (y_dist < 0) { // Potrzebny ruch w dół (Y-)
        if (robotPosition.rot == 0) {
          CurrentAction = RotateLeft; // Obrót o 180°
        }
        else if (robotPosition.rot == 90) {
          CurrentAction = RotateRight; // Obrót w prawo (90° -> 180°)
        }
        else if (robotPosition.rot == -90) {
          CurrentAction = RotateLeft; // Obrót w lewo (-90° -> 180°)
        }
      }
    } else { // Priorytet dla osi X
      if (x_dist > 0) { // Potrzebny ruch w prawo (X+)
        if (robotPosition.rot == 0) {
          CurrentAction = RotateRight; // Obrót w prawo (0° -> 90°)
        }
        else if (robotPosition.rot == 180) {
          CurrentAction = RotateLeft; // Obrót w lewo (180° -> 90°)
        }
        else if (robotPosition.rot == -90) {
          CurrentAction = RotateLeft; // Obrót o 180°
        }
      }
      else if (x_dist < 0) { // Potrzebny ruch w lewo (X-)
        if (robotPosition.rot == 0) {
          CurrentAction = RotateLeft; // Obrót w lewo (0° -> -90°)
        }
        else if (robotPosition.rot == 180) {
          CurrentAction = RotateRight; // Obrót w prawo (180° -> -90°)
        }
        else if (robotPosition.rot == 90) {
          CurrentAction = RotateLeft; // Obrót o 180°
        }
      }
    } 
    opponentAhead = false;
  }
}

// Setting gripper up and down
void gripper(int direction) {
  if (direction == 0){
    for(int i = 0; i<200; i++){
      digitalWrite(SERWO_A, HIGH);
      digitalWrite(SERWO_B, HIGH);
      delayMicroseconds(750);
      digitalWrite(SERWO_A, LOW);
      delayMicroseconds(1400);
      digitalWrite(SERWO_B, LOW);
      delayMicroseconds(17850);
    }
  } else {
    for(int i = 0; i<200; i++){
      digitalWrite(SERWO_A, HIGH);
      digitalWrite(SERWO_B, HIGH);
      delayMicroseconds(1300);
      digitalWrite(SERWO_B, LOW);
      delayMicroseconds(300);
      digitalWrite(SERWO_A, LOW);
      delayMicroseconds(18400);
    }
  }
}



void setup() {
  Serial.begin(9600);

  pinMode(L_DIR, OUTPUT);
  pinMode(R_DIR, OUTPUT);
  pinMode(L_STEP, OUTPUT);
  pinMode(R_STEP, OUTPUT);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }

  makeMeasurementsQueue=xQueueCreate(1, sizeof(int));


  xTaskCreate( // Function to adjust motor speed based on line tracking sensors
    FlagStep, "FlagStep", 1024, NULL, 1, NULL);

  xTaskCreate( // Function to make measurements
    MakeMeasurements, "MakeMeasurements", 512, NULL, 1, NULL);

  xTaskCreate( // Function to adjust motor speed based on line tracking sensors
    DisplayToSerial, "DisplayToSerial", 256, NULL, 1, NULL);

  xTaskCreate( // Function to adjust motor speed based on line tracking sensors
    OpponentDetection, "OpponentDetection", 256, NULL, 1, NULL);
}

void DisplayToSerial(void *pvParameters) {
  for (;;){
    Serial.print(CurrentAction);
    Serial.print(";");
    Serial.print(keepLineOn);
    Serial.print(";");
    Serial.print(robotPosition.posX);
    Serial.print(";");
    Serial.print(robotPosition.posY);
    Serial.print(";");
    Serial.print(robotPosition.rot);
    Serial.print(";");
    Serial.print(n);
    Serial.print(";");
    Serial.print(n_coeff);
    Serial.print(";");
    // Serial.print(readUltra1());
    // Serial.print(";");  
    // Serial.print(readUltra2());
    // Serial.print(";");  
    // Serial.print(readUltra3());

    Serial.println(";");  
    // for (int x=4;x>=0;x--){
    //   for (int y=0;y<5;y++){
    //   Serial.print(cantrix[x][y]);
    //   Serial.print(";");
    //   }
    //   Serial.println(";");
    // }
    vTaskDelay(1000/portTICK_PERIOD_MS);
  }
}

void loop() {}

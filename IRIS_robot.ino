#include <Stepper.h> 
#include <Arduino_FreeRTOS.h>

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
int frontLineSensorPins[] = {C4, C5, C6, C7, C8}; //Left to right 
// int frontLineSensorPins[] = {C8, C7, C6, C5, C4};
int backLineSensorPins[] = {C16, C15, C14, C13, C12}; // Left to right
// int backLineSensorPins[] = {C12, C13, C14, C15, C16};
int leftLineSensorPins[] = {C3, C2, C1};
int rightLineSensorPins[] = {C9, C10, C11};

//Stepper motors
#define R_STEP 51
#define L_STEP 50
#define R_DIR 53
#define L_DIR 52
#define MICROSTEP 16
#define Td 120

// Function to read sensor value and return a boolean state
bool readSensor(int sensPin){
  return (analogRead(sensPin) > THRESHOLD);
}

// Function prototypes for FreeRTOS tasks
void FlagStep(void *pvParameters);

// Other functions used by a robot 
void rotateBy(int angle, bool sensored = true);

// Motor control flags
int remainingCrossings = 4; //Crossings remaining, 0 - stop at the next crossing
bool keepLineOn = false; //flag for detecting crossing
unsigned int previousCrossingTimestamp = millis(); // Timer for detecting crossing in case of crossing the crossing not straight 

// Motor action settings
enum RobotAction {Straighten=0, RotateLeft=-90, RotateRight=90};
RobotAction CurrentAction = Straighten;

//Misc
#define BUZZER 8

// Structure to store robot position and orientation
struct coordinates {
  int posX = 2;
  int posY = 0;
  int rot = 0;  // 0-forw, 90-right, 180-back, -90-left
};

bool cantrix[5][5] = {{0,0,0,0,0},{0,0,0,0,0},{0,1,0,1,0},{0,0,0,0,0},{0,0,0,0,0}};

coordinates robotPosition;

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
int ultra1, ultra2, ultra3 = 0;
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
    // Serial.print(CurrentAction);
    // Serial.print(";");
    // Serial.print(remainingCrossings);
    // Serial.println(";");
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
      }
        else{
          bothMotorStep(3);
        }

      //Below is intersection detection
      unsigned int tempTime = abs(millis() - previousCrossingTimestamp);
      if ((readSensor(rightLineSensorPins[2]) || readSensor(leftLineSensorPins[2])) && keepLineOn == false ) {
        keepLineOn = true;
        previousCrossingTimestamp = millis();
        if (remainingCrossings == 0) {
          CurrentAction = RotateRight;
        } 
        if (robotPosition.rot == 0){
          robotPosition.posY = robotPosition.posY + 1;
        }
        else if (robotPosition.rot == 180){
          robotPosition.posY = robotPosition.posY - 1;
        }
        else if (robotPosition.rot == 90){
          robotPosition.posX = robotPosition.posX + 1;
        }
        else if (robotPosition.rot == -90){
          robotPosition.posX = robotPosition.posX - 1;
        }

        remainingCrossings-=1; // Decrementation after crossing the intersection
      } else if (!readSensor(rightLineSensorPins[2]) && !readSensor(leftLineSensorPins[2]) && tempTime>=500) {
        keepLineOn = false;
      }
    }
  } else {
    for(int i = 0; i < stepCount; i++){
      if(readSensor(C14)){
        bothMotorStep(12);
      } else if(readSensor(C15)){
        for(int j = 0; j < 4; j++){
          bothMotorStep(2);
          leftMotorStep(1);
        }
      } else if(readSensor(C13)){
        for(int j = 0; j < 4; j++){
          bothMotorStep(2);
          rightMotorStep(1);
        }
      } else if(readSensor(C16)){
        for(int j = 0; j < 6; j++){
          bothMotorStep(1);
          leftMotorStep(1);
        }
      } else if(readSensor(C12)){
        for(int j = 0; j < 6; j++){
          bothMotorStep(1);
          rightMotorStep(1);
        }
      }
      //Logic for driving backwards should be placed below
    }
  }

}

void rotateBy(int angle, bool sensored = true){
    // Serial.print(CurrentAction);
    // Serial.println(";");
  
  if(angle > 0){
    digitalWrite(L_DIR, LOW);
    digitalWrite(R_DIR, HIGH);
  }else{
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
  }
  else if (angle == 90 && robotPosition.rot == 180){
    robotPosition.rot = -90;
  }
  else if (angle == -90 && robotPosition.rot != -90){
    robotPosition.rot = robotPosition.rot + angle;
  }
  else if (angle == -90 && robotPosition.rot == -90){
    robotPosition.rot = 180;
  } 
  else if (angle == 180 && robotPosition.rot == 90){
    robotPosition.rot = -90;
  }
  else if (angle == 180 && robotPosition.rot == 180){
    robotPosition.rot = 0;
  }
  else if (angle == 180 && (robotPosition.rot == 0 || robotPosition.rot ==-90)){
    robotPosition.rot = robotPosition.rot + angle;
  }

  keepLineOn=true;
  previousCrossingTimestamp = millis();
  remainingCrossings=1;
  CurrentAction=Straighten;

}
void moveStraightUntilSides(){
  digitalWrite(R_DIR, LOW);
  digitalWrite(L_DIR, LOW);
  while(!readSensor(C2) || !readSensor(C10)){
    bothMotorStep(1);
  }
  while(!readSensor(C2)){
    rightMotorStep(1);
  }
  while(!readSensor(C10)){
    leftMotorStep(1);
  }
  delay(250);
}

// Function to adjust motor speed based on line tracking sensors
void FlagStep(void *pvParameters)
{
for (;;){
     // Serial.print(readSharp1());
  // Serial.print(";");
  // Serial.print(readSharp2());
  // Serial.print(";");
  // Serial.print(readSharp3());
  // Serial.println(";");
    if (CurrentAction == Straighten) {
      moveStraight(10);
    } else if (CurrentAction == RotateRight ){
      rotateBy(90);
    } else if (CurrentAction == RotateLeft ){
      rotateBy(-90);
    }
  int Sharp1 = readSharp1();
  int Sharp2 = readSharp2();
  int Sharp3 = readSharp3();
  // Serial.print(Sharp1);
  // Serial.print(";");
  // Serial.print(Sharp2);
  // Serial.print(";");
  // Serial.print(Sharp3);
  // Serial.println(";");
    if (keepLineOn) {
      if (Sharp1 >= 2 && robotPosition.rot == 0){
        cantrix[robotPosition.posY][robotPosition.posX-1] = 1;
      } else if (Sharp1 >= 2 && robotPosition.rot == 180){
        cantrix[robotPosition.posY][robotPosition.posX+1] = 1;
      } else if (Sharp1 >= 2 && robotPosition.rot == 90){
        cantrix[robotPosition.posY+1][robotPosition.posX] = 1;
      } else if (Sharp1 >= 2 && robotPosition.rot == -90){
        cantrix[robotPosition.posY-1][robotPosition.posX] = 1;
      }

      if (Sharp3 >= 2 && robotPosition.rot == 0){
        cantrix[robotPosition.posY][robotPosition.posX+1] = 1;
      } else if (Sharp3 >= 2 && robotPosition.rot == 180){
        cantrix[robotPosition.posY][robotPosition.posX-1] = 1;
      } else if (Sharp3 >= 2 && robotPosition.rot == 90){
        cantrix[robotPosition.posY-1][robotPosition.posX] = 1;
      } else if (Sharp3 >= 2 && robotPosition.rot == -90){
        cantrix[robotPosition.posY+1][robotPosition.posX] = 1;
      }

      
      if (Sharp2 >= 2 && robotPosition.rot == 0){
        cantrix[robotPosition.posY+1][robotPosition.posX] = 1;
      } else if (Sharp2 >= 2 && robotPosition.rot == 180){
        cantrix[robotPosition.posY-1][robotPosition.posX] = 1;
      } else if (Sharp2 >= 2 && robotPosition.rot == 90){
        cantrix[robotPosition.posY][robotPosition.posX+1] = 1;
      } else if (Sharp2 >= 2 && robotPosition.rot == -90){
        cantrix[robotPosition.posY][robotPosition.posX-1] = 1;
      }

    }
    vTaskDelay(10/portTICK_PERIOD_MS);
  }
}


void DisplayCans(void *pvParameters) {
  for (;;){
    Serial.print(robotPosition.posX);
    Serial.print(";");
    Serial.print(robotPosition.posY);
    Serial.print(";");
    Serial.print(robotPosition.rot);
    Serial.println(";");  
    for (int x=4;x>=0;x--){
      for (int y=0;y<5;y++){
      Serial.print(cantrix[x][y]);
      Serial.print(";");
      }
      Serial.println(";");
    }
    vTaskDelay(1000/portTICK_PERIOD_MS);
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

  xTaskCreate( // Function to adjust motor speed based on line tracking sensors
    FlagStep, "FlagStep", 256, NULL, 1, NULL);

  xTaskCreate( // Function to adjust motor speed based on line tracking sensors
    DisplayCans, "DisplayCans", 256, NULL, 1, NULL);

}

void loop() {
  // for (int i = 0; i <= 15; i++) {
  //   Serial.print("Sharp 1 = ");
  //   Serial.print(readSharp1());  
  //   Serial.print(", ");
  //   Serial.print("Sharp 2 = ");
  //   Serial.print(readSharp2());  
  //   Serial.print(", ");
  //   Serial.print("Sharp 3 = ");
  //   Serial.print(readSharp3());  
  //   Serial.print(", ");
  //   Serial.print("Ulttra 1 = ");
  //   Serial.print(readUltra1());  
  //   Serial.print(", ");
  //   Serial.print("Ultra 2 = ");
  //   Serial.print(readUltra2());  
  //   Serial.print(", ");
  //   Serial.print("Ultra 3 = ");
  //   Serial.print(readUltra3());  
  //   Serial.print("\n");
  // }


  // put your main code here, to run repeatedly:

}

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
#define THRESHOLD 400 //value of left back sensor is lowered artificially
// #define THRESHOLD 600

// Arrays storing pin numbers for line sensors
int frontLineSensorPins[] = {C4, C5, C6, C7, C8};
// int frontLineSensorPins[] = {C8, C7, C6, C5, C4};
// int backLineSensorPins[] = {C16, C15, C14, C13, C12};
int backLineSensorPins[] = {C12, C13, C14, C15, C16};
int leftLineSensorPins[] = {C3, C2, C1};
int rightLineSensorPins[] = {C9, C10, C11};
int *lineSensors[] = {frontLineSensorPins, backLineSensorPins};

//Stepper motors
#define R_STEP 51
#define L_STEP 50
#define R_DIR 53
#define L_DIR 52
#define MICROSTEP 16
#define Td 90

// Function to read sensor value and return a boolean state
bool readSensor(int sensPin){
  return (analogRead(sensPin) > THRESHOLD);
}

// Stepper motor objects
// Stepper myStepperLeft = Stepper(200, L_STEP, L_DIR);
// Stepper myStepperRight = Stepper(200, R_STEP, R_DIR);

// Function prototypes for FreeRTOS tasks
void StepperRight( void *pvParameters );
void StepperLeft( void *pvParameters );
void FlagStep(void *pvParameters);

// Motor control flags
int remainingCrossings = 3; //Crossings remaining, 0 - stop at the next crossing
bool keepLineOn = false; //flag for detecting crossing
unsigned int previousCrossingTimestamp = millis(); // Timer for detecting crossing in case of crossing the crossing not straight 

// Motor step speed settings
enum MotorStep {LowStep=95, MediumStep=80, MaxStep=65};
enum RobotAction {Straighten=0, RotateLeft=1, RotateRight=2};
MotorStep rightMotorStepCount = MaxStep;
MotorStep leftMotorStepCount = MaxStep;
RobotAction CurrentAction = Straighten;

// Rotation angle calc
float wheelRotations = (float)(90 / 360.0 * 2);
int32_t rotationStepCount = (int32_t)(wheelRotations * 200 * MICROSTEP);


//Misc
#define BUZZER 8

// Structure to store robot position and orientation
struct coordinates {
  int posX;
  int posY;
  int rot;  // 0-forw, 1- right, 2-back, 3-left
};

coordinates robotPosition;

unsigned long elapsedTime;

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

// Function controlling left stepper motor
void StepperLeft( void *pvParameters ) {
  (void) pvParameters;
  for (;;)
  {
    if (CurrentAction == Straighten) {
      digitalWrite(L_DIR, LOW);
      digitalWrite(L_STEP, HIGH);
      delayMicroseconds(leftMotorStepCount);
      digitalWrite(L_STEP, LOW);
      delayMicroseconds(leftMotorStepCount);
    } else if (CurrentAction == RotateRight) {

      digitalWrite(L_DIR, LOW);
      for(int x = 0; x < rotationStepCount; x++){
      // while(!((readSensor(rightLineSensorPins[0] || rightLineSensorPins[1] || rightLineSensorPins[2]) && readSensor(leftLineSensorPins[0] || leftLineSensorPins[1] || leftLineSensorPins[2] ))&& keepLineOn == false)){
        digitalWrite(L_STEP, HIGH);
        delayMicroseconds(300);
        digitalWrite(L_STEP, LOW);
        delayMicroseconds(300);
        if(readSensor(frontLineSensorPins[2]) && keepLineOn==false && x >=0.75*rotationStepCount) {break;}
      }
      keepLineOn=true;
      previousCrossingTimestamp = millis();
      remainingCrossings=1;
      CurrentAction=Straighten;

    } else if (CurrentAction == RotateLeft) {
      digitalWrite(L_DIR, HIGH);
      for(int x = 0; x < rotationStepCount; x++){
      // while(!((readSensor(rightLineSensorPins[0] || rightLineSensorPins[1] || rightLineSensorPins[2]) && readSensor(leftLineSensorPins[0] || leftLineSensorPins[1] || leftLineSensorPins[2] ))&& keepLineOn == false)){
        digitalWrite(L_STEP, HIGH);
        delayMicroseconds(300);
        digitalWrite(L_STEP, LOW);
        delayMicroseconds(300);
        if(readSensor(frontLineSensorPins[2]) && (keepLineOn==false || x >=0.75*rotationStepCount)) {break;}
      }
      keepLineOn=true;
      previousCrossingTimestamp = millis();
      remainingCrossings=1;
      CurrentAction=Straighten;
    }
  }

}


// Function controlling left stepper motor
void StepperRight( void *pvParameters ) {
  (void) pvParameters;
  for (;;)
  {
    if (CurrentAction == Straighten) {
      digitalWrite(R_DIR, LOW);
      digitalWrite(R_STEP, HIGH);
      delayMicroseconds(rightMotorStepCount);
      digitalWrite(R_STEP, LOW);
      delayMicroseconds(rightMotorStepCount);
      
    } else if (CurrentAction == RotateRight) {
      digitalWrite(R_DIR, HIGH);
      for(int x = 0; x < rotationStepCount; x++){
      // while(!((readSensor(rightLineSensorPins[0]) || readSensor(rightLineSensorPins[1]) || readSensor(rightLineSensorPins[2]) && readSensor(leftLineSensorPins[0]) || readSensor(leftLineSensorPins[1]) || readSensor(leftLineSensorPins[2]))&& keepLineOn == false)){
        digitalWrite(R_STEP, HIGH);
        delayMicroseconds(300);
        digitalWrite(R_STEP, LOW);
        delayMicroseconds(300);
        if(readSensor(frontLineSensorPins[2]) && keepLineOn==false && x >=0.75*rotationStepCount) {break;}
      }
      keepLineOn=true;
      previousCrossingTimestamp = millis();
      remainingCrossings=1;
      CurrentAction=Straighten;
    } else if (CurrentAction == RotateLeft) {
      for(int x = 0; x < rotationStepCount; x++){
      // while(!((readSensor(rightLineSensorPins[0]) || readSensor(rightLineSensorPins[1]) || readSensor(rightLineSensorPins[2]) && readSensor(leftLineSensorPins[0]) || readSensor(leftLineSensorPins[1]) || readSensor(leftLineSensorPins[2]))&& keepLineOn == false)){
        digitalWrite(R_DIR, LOW);
        digitalWrite(R_STEP, HIGH);
        delayMicroseconds(300);
        digitalWrite(R_STEP, LOW);
        delayMicroseconds(300);
        if(readSensor(frontLineSensorPins[2]) && (keepLineOn==false || x >=0.75*rotationStepCount)) {
          Serial.print("wykonano");
          break;
        }
      }
      keepLineOn=true;
      previousCrossingTimestamp = millis();
      remainingCrossings=1;
      CurrentAction=Straighten;
    }
  }

}

// Function to adjust motor speed based on line tracking sensors
void FlagStep(void *pvParameters)
{
for (;;)
  {
    if (CurrentAction == Straighten) {
      int frontSensors=0, backSensors=0;
      for (int i =0;i<=5;i++) {
        frontSensors+=readSensor(frontLineSensorPins[i]);
        backSensors+=readSensor(backLineSensorPins[i]);
      }

      int priority=0;
      if (frontSensors!=0 && frontSensors<backSensors) {
        priority = 0;
      } else if (backSensors!=0 && backSensors<frontSensors){
        priority = 1;
      } else if (backSensors==frontSensors) {
        priority = 0;
      }

      int offset = 10; 
      if (readSensor(frontLineSensorPins[2]) && readSensor(backLineSensorPins[2])) {
        leftMotorStepCount = MaxStep + offset;
        rightMotorStepCount = MaxStep;
        Serial.print(0);
        Serial.print(";");
      } else if (readSensor(frontLineSensorPins[1]) || readSensor(backLineSensorPins[1]) && !(readSensor(frontLineSensorPins[2]) && readSensor(backLineSensorPins[2]))) {
        leftMotorStepCount = MaxStep + offset;
        rightMotorStepCount = MediumStep;
        Serial.print(1);
        Serial.print(";");
      } else if (readSensor(frontLineSensorPins[3]) || readSensor(backLineSensorPins[3]) && !(readSensor(frontLineSensorPins[2]) && readSensor(backLineSensorPins[2]))) {
        leftMotorStepCount = MediumStep + offset;
        rightMotorStepCount = MaxStep;
        Serial.print(2);
        Serial.print(";");
      } else if (readSensor(frontLineSensorPins[0]) || readSensor(backLineSensorPins[4])) {
        leftMotorStepCount = LowStep + offset;
        rightMotorStepCount = MaxStep;
        Serial.print(3);
        Serial.print(";");
      } else if (readSensor(frontLineSensorPins[4]) || readSensor(backLineSensorPins[0])) {
        leftMotorStepCount = MaxStep + offset;
        rightMotorStepCount = LowStep;
        Serial.print(4);
        Serial.print(";");
      }
      // Serial.print(priority);
      // int v = 0;
      // if (readSensor(lineSensors[priority][2])){
      //   leftMotorStepCount = MaxStep-v;
      //   rightMotorStepCount = MaxStep;
      // }else if (readSensor(lineSensors[priority][1])) {
      //   leftMotorStepCount = MediumStep-v;
      //   rightMotorStepCount = MaxStep;
      // } else  if (readSensor(lineSensors[priority][3])) {
      //   leftMotorStepCount = MaxStep-v;
      //   rightMotorStepCount = MediumStep;
      // } else if (readSensor(lineSensors[priority][0]) ) {
      //   leftMotorStepCount = LowStep -v;
      //   rightMotorStepCount = MaxStep;
      // } else if (readSensor(lineSensors[priority][4])){
      //   leftMotorStepCount = MaxStep-v;
      //   rightMotorStepCount = LowStep;
      // }
    }

    //Below is intersection detection
    unsigned int tempTime = abs(millis() - previousCrossingTimestamp);
    if ((readSensor(rightLineSensorPins[2]) || readSensor(leftLineSensorPins[2])) && keepLineOn == false ) {
      keepLineOn = true;
      previousCrossingTimestamp = millis();
      if (remainingCrossings == 0) {
        CurrentAction = RotateLeft;
      } 
      remainingCrossings-=1; // Decrementation after crossing the intersection
    } else if (!readSensor(rightLineSensorPins[2]) && !readSensor(leftLineSensorPins[2]) && tempTime>=500) {
      keepLineOn = false;
    }
    // Serial.print(";");
    Serial.print(CurrentAction);
    Serial.print(";");
    // Serial.println(remainingCrossings);
    // Serial.print(";");
    // Serial.print(priority);
    // Serial.print(";");
    Serial.print(leftMotorStepCount);
    Serial.print(";");
    Serial.println(rightMotorStepCount);
    vTaskDelay(10/portTICK_PERIOD_MS);
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
  
  xTaskCreate( // Function controlling left stepper motor
    StepperLeft, "StepperLeft", 256, NULL, 1, NULL);

  xTaskCreate( // Function controlling left stepper motor
    StepperRight, "StepperRight", 256, NULL, 1, NULL);

  xTaskCreate( // Function to adjust motor speed based on line tracking sensors
    FlagStep, "FlagStep", 256, NULL, 1, NULL);

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

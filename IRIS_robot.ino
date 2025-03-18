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
#define THRESHOLD 400

// Arrays storing pin numbers for line sensors
int frontLineSensorPins[] = {C4, C5, C6, C7, C8};
int backLineSensorPins[] = {C16, C15, C14, C13, C12};
int leftLineSensorPins[] = {C3, C2, C1};
int rightLineSensorPins[] = {C9, C10, C11};
int *lineSensors[] = {frontLineSensorPins, backLineSensorPins};

//Stepper motors
#define R_STEP 51
#define L_STEP 50
#define R_DIR 53
#define L_DIR 52
#define MICROSTEP 16
#define Td 180

// Function to read sensor value and return a boolean state
bool readSensor(int sensPin){
  return (analogRead(sensPin) > THRESHOLD);
}

// Stepper motor objects
Stepper myStepperLeft = Stepper(200, L_STEP, L_DIR);
Stepper myStepperRight = Stepper(200, R_STEP, R_DIR);

// Function prototypes for FreeRTOS tasks
void StepperRight( void *pvParameters );
void StepperLeft( void *pvParameters );
void FlagStep(void *pvParameters);

// Motor control flags
bool enableRightMotor = true;
bool enableLeftMotor = true;
int remainingCrossings = 2;
bool keepLineOn = false;
bool haltAtNextCrossing = false;

// Motor step speed settings
enum MotorStep {LowStep=190 ,MediumStep=200,MaxStep=210};
MotorStep rightMotorStepCount = MaxStep;
MotorStep leftMotorStepCount = MaxStep;

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

void setup() {
  Serial.begin(9600);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }
  
  xTaskCreate( // Function controlling left stepper motor
      StepperLeft
      ,  "StepperLeft"
      ,  128
      ,  NULL
      ,  1
      ,  NULL );


  xTaskCreate( // Function controlling right stepper motor
    StepperRight
    ,  "StepperRight"
    ,  128
    ,  NULL
    ,  1
    ,  NULL );

  xTaskCreate( // Function to adjust motor speed based on line tracking sensors
    FlagStep
    ,  "FlagStep"
    ,  128
    ,  NULL
    ,  1
    ,  NULL );

}

// Function controlling right stepper motor
void StepperRight( void *pvParameters ) {
  (void) pvParameters;
  for (;;)
  {
    if (enableRightMotor) {
      myStepperRight.step(rightMotorStepCount);
      vTaskDelay(1);
    }
  }
}

// Function controlling left stepper motor
void StepperLeft( void *pvParameters ) {
  (void) pvParameters;
  for (;;)
  {
    if (enableLeftMotor) {
      myStepperLeft.step(leftMotorStepCount);
      vTaskDelay(1);
    }
  }

}

// Function to adjust motor speed based on line tracking sensors
void FlagStep(void *pvParameters)
{
for (;;)
  {
    if (haltAtNextCrossing==false) {
      int sumFront = 0, sumBack = 0;
      for(int i=0; i<=4; i++)
      {
        sumFront+=readSensor(frontLineSensorPins[i]);
        sumBack+=readSensor(backLineSensorPins[i]);
      }
      int priority;
      if (sumFront==0) {
        priority = 1;
        
      } else if (sumBack == 0){
        priority=0;
      } else {
        if (sumFront>sumBack+1) {
          priority=1;
        } else {
          priority=0;
        }
      }

      if (readSensor(lineSensors[priority][0]) ) {
        leftMotorStepCount = LowStep -10;
        rightMotorStepCount = MaxStep;
        
      }
      else if (readSensor(lineSensors[priority][1])) {
        leftMotorStepCount = MediumStep-10;
        rightMotorStepCount = MaxStep;
      }
      else  if (readSensor(lineSensors[priority][3])) {
        leftMotorStepCount = MaxStep-10;
        rightMotorStepCount = MediumStep;
      }
      else if (readSensor(lineSensors[priority][4])){
        leftMotorStepCount = MaxStep-10;
        rightMotorStepCount = LowStep;
      } else if (readSensor(lineSensors[priority][2])){
        leftMotorStepCount = MaxStep-10;
        rightMotorStepCount = MaxStep;
      }
    }
    //Below is intersection detection
    Serial.print(remainingCrossings);
    Serial.print("\n");

    if ((readSensor(rightLineSensorPins[0]) || readSensor(leftLineSensorPins[0])) && keepLineOn == false) {
      keepLineOn = true;
      remainingCrossings-=1;
      if (remainingCrossings == 0) {
        haltAtNextCrossing = true;
        enableLeftMotor = false;
        enableRightMotor = false;
      } 
    } else if (!readSensor(rightLineSensorPins[0]) && !readSensor(leftLineSensorPins[0])) {
      keepLineOn = false;
    }

    vTaskDelay(10/portTICK_PERIOD_MS);
  }
}

void stop() {

}

// 3 in Ultra means there is something really close
// 3 in Sharp means there is opponent close

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

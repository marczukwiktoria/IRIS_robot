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
 
int frontLineSensors[] = {C4, C5, C6, C7, C8};
int backLineSensors[] = {C12, C13, C14, C15, C16};
int *lineSensors[] = {frontLineSensors, backLineSensors};

//Stepper motors
#define R_STEP 51
#define L_STEP 50
#define R_DIR 53
#define L_DIR 52
#define MICROSTEP 16
#define Td 180

bool readSensor(int sensPin){
  return (analogRead(sensPin) > THRESHOLD);
}

Stepper myStepperLeft = Stepper(200, L_STEP, L_DIR);
Stepper myStepperRight = Stepper(200, R_STEP, R_DIR);

void StepperRight( void *pvParameters );
void StepperLeft( void *pvParameters );
void FlagStep(void *pvParameters);
int flagRight = 1;
int flagLeft = 1;

enum MotorStep {LowStep=0 ,MediumStep=50,MaxStep=100};

MotorStep flagStepRight = MaxStep;
MotorStep flagStepLeft = MaxStep;



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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }

  xTaskCreate(
      StepperLeft
      ,  "StepperLeft"   // A name just for humans
      ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
      ,  NULL
      ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,  NULL );

  xTaskCreate(
    StepperRight
    ,  "StepperRight"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2
    ,  NULL );

  xTaskCreate(
    FlagStep
    ,  "FlagStep"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  0
    ,  NULL );


}



void StepperRight( void *pvParameters ) {
  (void) pvParameters;
  for (;;)
  {
    if (flagRight) {
      //Serial.print(String(flagStepRight));
      //Serial.print("R\n");

      myStepperRight.step(flagStepRight);
      vTaskDelay(1);
    }
  }
}

void StepperLeft( void *pvParameters ) {
  (void) pvParameters;
  for (;;)
  {
    if (flagLeft) {
      
      //Serial.print(String(flagStepLeft));
      //Serial.print("L\n");
      
      myStepperLeft.step(flagStepLeft);
      vTaskDelay(1);
    }
  }

}

void FlagStep(void *pvParameters)
{
for (;;)
  {
    int sumFront = 0, sumBack = 0;
    for(int i=0; i<=4; i++)
    {
      sumFront+=readSensor(frontLineSensors[i]);
      sumBack+=readSensor(backLineSensors[i]);
    }
    int priority;
    if (sumFront>sumBack+1) {
      priority=1;
    } else {
      priority=0;
    }


    if (readSensor(lineSensors[priority][0]) ) {
      flagStepLeft = LowStep;
      flagStepRight = MaxStep;
      
    }
    else if (readSensor(lineSensors[priority][1])) {
      flagStepLeft = MediumStep;
      flagStepRight = MaxStep;
    }
    else if (readSensor(lineSensors[priority][2])){
      flagStepLeft = MaxStep;
      flagStepRight = MaxStep;
    }
    else  if (readSensor(lineSensors[priority][3])) {
      flagStepLeft = MaxStep;
      flagStepRight = MediumStep;
    }
    else if (readSensor(lineSensors[priority][4])){
      flagStepLeft = MaxStep;
      flagStepRight = LowStep;
    }


    
  vTaskDelay(100/portTICK_PERIOD_MS); 
  }
 
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

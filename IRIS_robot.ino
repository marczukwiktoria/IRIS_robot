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

}

void loop() {
  for (int i = 0; i <= 15; i++) {
    Serial.print("Sharp 1 = ");
    Serial.print(readSharp1());  
    Serial.print(", ");
    Serial.print("Sharp 2 = ");
    Serial.print(readSharp2());  
    Serial.print(", ");
    Serial.print("Sharp 3 = ");
    Serial.print(readSharp3());  
    Serial.print(", ");
    Serial.print("Ulttra 1 = ");
    Serial.print(readUltra1());  
    Serial.print(", ");
    Serial.print("Ultra 2 = ");
    Serial.print(readUltra2());  
    Serial.print(", ");
    Serial.print("Ultra 3 = ");
    Serial.print(readUltra3());  
    Serial.print("\n");
    delay(1000);
  }
  // put your main code here, to run repeatedly:

}

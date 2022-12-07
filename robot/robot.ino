#include <IRremote.h>
#define IR_PIN 5    //IR receiver Signal pin connect to Arduino pin D10
IRrecv IR(IR_PIN);  //   IRrecv object  IR get code from IR remoter
decode_results IRresults;
#define IR_ADVANCE 0x00FF18E7    //code from IR controller "▲" button
#define IR_BACK 0x00FF4AB5       //code from IR controller "▼" button
#define IR_RIGHT 0x00FF5AA5      //code from IR controller ">" button
#define IR_LEFT 0x00FF10EF       //code from IR controller "<" button
#define IR_STOP 0x00FF38C7       //code from IR controller "OK" button
#define IR_TANKLEFT 0x00FF6897   //code from IR controller "#" button
#define IR_TANKRIGHT 0x00FFB04F  //code from IR controller "#" button
#define IR_KLAXON 0x00FF9867     //code from IR controller "#" button
#define speedPinR 9              //  RIGHT PWM pin connect MODEL-X ENA
#define RightMotorDirPin1 12     //Right Motor direction pin 1 to MODEL-X IN1
#define RightMotorDirPin2 11     //Right Motor direction pin 2 to MODEL-X IN2
#define speedPinL 6              // Left PWM pin connect MODEL-X ENB
#define LeftMotorDirPin1 7       //Left Motor direction pin 1 to MODEL-X IN3
#define LeftMotorDirPin2 8       //Left Motor direction pin 1 to MODEL-X IN4
#define UltrasonicSensorInput 10
#define UltrasonicSensorOutput 2

enum COMMAND {
  STOP,
  ADVANCE,
  BACK,
  LEFT,
  TANKLEFT,
  RIGHT,
  TANKRIGHT,
  KLAXON,
};

COMMAND state;

volatile int distance;

void do_IR_Tick() {
  if (IR.decode(&IRresults)) {
    if (IRresults.value == IR_ADVANCE) {
      Serial.println("avant");
      state = ADVANCE;
    } else if (IRresults.value == IR_RIGHT) {
      Serial.println("droite");
      state = RIGHT;
    } else if (IRresults.value == IR_LEFT) {
      Serial.println("gauche");
      state = LEFT;
    } else if (IRresults.value == IR_BACK) {
      Serial.println("arrière");
      state = BACK;
    } else if (IRresults.value == IR_STOP) {
      Serial.println("stop");
      state = STOP;
    } else if (IRresults.value == IR_TANKLEFT) {
      Serial.println("tankleft");
      state = TANKLEFT;
    } else if (IRresults.value == IR_TANKRIGHT) {
      Serial.println("tankright");
      state = TANKRIGHT;
    } else if (IRresults.value == IR_KLAXON) {
      Serial.println("KLAXON");
      state = KLAXON;
    } else {
      //Serial.print("code : ");
      Serial.println(IRresults.value, HEX);
    }
    IRresults.value = 0;
    IR.resume();
  }
}

void move(int speedL, int speedR, bool FR, bool RR, bool FL, bool RL) {
  analogWrite(speedPinL, speedL);
  analogWrite(speedPinR, speedR);
  digitalWrite(RightMotorDirPin1, FR ? HIGH : LOW);
  digitalWrite(RightMotorDirPin2, RR ? HIGH : LOW);
  digitalWrite(LeftMotorDirPin1, FL ? HIGH : LOW);
  digitalWrite(LeftMotorDirPin2, RL ? HIGH : LOW);
}

float checkdistance() {
  digitalWrite(UltrasonicSensorInput, LOW);
  delayMicroseconds(2);
  digitalWrite(UltrasonicSensorInput, HIGH);
  delayMicroseconds(10);
  digitalWrite(UltrasonicSensorInput, LOW);
  float distance = pulseIn(UltrasonicSensorOutput, HIGH);
  //float distance = pulseIn(UltrasonicSensorOutput, HIGH) / 58.00;
  delay(10);
  return distance;
}

void setup() {
  distance = 0;
  pinMode(UltrasonicSensorOutput, OUTPUT);
  pinMode(UltrasonicSensorInput, INPUT);
  pinMode(IR_PIN, INPUT);
  digitalWrite(IR_PIN, HIGH);
  IR.enableIRIn();
  Serial.begin(9600);
  state = STOP;
}

void loop() {
  distance = checkdistance();
  Serial.println(distance);
  do_IR_Tick();
  switch (state) {
    case STOP:
      move(0,0,false,false,false,false);
      //move(200,200,true,false,true,false);
      break;
    case ADVANCE:
      move(511,511,true,false,true,false);
      break;
    case BACK:
      move(200,200,false,true,false,true);
      break;
    case LEFT:
      move(0,200,true,false,true,false);
      break;
    case TANKLEFT:
      move(150,150,true,false,false,true);
      break;
    case RIGHT:
      move(200,0,true,false,true,false);
      break;
    case TANKRIGHT:
      move(150,150,false,true,true,false);
      break;
    case KLAXON:
        move(511,511,false,true,true,false);
        delay(100);
        move(511,511,true,false,false,true);
        delay(100);
      break;
  }
  delay(20);
}

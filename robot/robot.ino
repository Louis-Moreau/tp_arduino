#include <IRremote.hpp>
#include <Servo.h>
#define IR_PIN 3            //IR receiver Signal pin connect to Arduino pin D10
#define speedPinR 5         // RIGHT PWM pin connect MODEL-X ENA
#define RightDirectPin1 12  //  Right Motor direction pin 1 to MODEL-X IN1
#define RightDirectPin2 11  // Right Motor direction pin 2 to MODEL-X IN2
#define speedPinL 6         //  Left PWM pin connect MODEL-X ENB
#define LeftDirectPin1 7    // Left Motor direction pin 1 to MODEL-X IN3
#define LeftDirectPin2 8    //Left Motor direction pin 1 to MODEL-X IN4
#define UltrasonicTrigPin 10
#define UltrasonicEchoPin 2

#define servoPin 9
#define BuzzerPin 13

#define IR_ADVANCE 0x18   //code from IR controller "▲" button
#define IR_BACK 0x52      //code from IR controller "▼" button
#define IR_RIGHT 0X5A     //code from IR controller ">" button
#define IR_LEFT 0x8       //code from IR controller "<" button
#define IR_STOP 0x1C      //code from IR controller "OK" button
#define IR_TANKLEFT 0x16  //code from IR controller "#" button
#define IR_TANKRIGHT 0xD  //code from IR controller "#" button
#define IR_TWERK 0x19
#define IR_KLAXON 0x46
#define IR_AUTO 0x45  //code from IR controller "#" button


enum COMMAND {
  STOP,
  ADVANCE,
  BACK,
  LEFT,
  TANKLEFT,
  RIGHT,
  TANKRIGHT,
  KLAXON,
  AUTO,
  TWERK
};

enum DIRECTION {
  GOLEFT,
  GOFRONT,
  GORIGHT,
  CONTINUE
};

COMMAND state;
Servo myservo;

void autoTick();
void do_IR_Tick();
float checkdistance();
void move(int speedL, int speedR, bool FR, bool RR, bool FL, bool RL);

void setup() {
  pinMode(UltrasonicTrigPin, OUTPUT);
  pinMode(UltrasonicEchoPin, INPUT);

  pinMode(RightDirectPin1, OUTPUT);
  pinMode(RightDirectPin2, OUTPUT);
  pinMode(LeftDirectPin1, OUTPUT);
  pinMode(LeftDirectPin2, OUTPUT);
  pinMode(speedPinR, OUTPUT);
  pinMode(speedPinL, OUTPUT);

  pinMode(BuzzerPin, OUTPUT);
  IrReceiver.begin(IR_PIN);
  myservo.attach(servoPin);

  randomSeed(analogRead(0));
  Serial.begin(9600);
  state = STOP;
}

void loop() {
  //distance = checkdistance();
  //Serial.println(distance);
  do_IR_Tick();
  switch (state) {
    case STOP:
      move(0, 0, false, false, false, false);
      //move(200,200,true,false,true,false);
      break;
    case ADVANCE:
      move(511, 511, true, false, true, false);
      break;
    case BACK:
      move(250, 250, false, true, false, true);
      break;
    case LEFT:
      move(0, 200, true, false, true, false);
      break;
    case TANKLEFT:
      move(200, 200, true, false, false, true);
      break;
    case RIGHT:
      move(200, 0, true, false, true, false);
      break;
    case TANKRIGHT:
      move(200, 200, false, true, true, false);
      break;
    case TWERK:
      move(511, 511, false, true, true, false);
      delay(100);
      move(511, 511, true, false, false, true);
      delay(80);
      break;
    case KLAXON:
      for (int i = 0; i < 15; i++) {
        tone(BuzzerPin, (500));
        delay(50);  // ...for 1 sec
        noTone(BuzzerPin);
        delay(25);
      }
      IrReceiver.begin(IR_PIN);
      state = STOP;
      break;
    case AUTO:
      autoTick();
      break;
  }
  delay(20);
}


void do_IR_Tick() {
  if (IrReceiver.decode()) {
    //IrReceiver.printIRResultShort(&Serial);
    if (IrReceiver.decodedIRData.protocol == NEC) {  // SAMSUNG
      if (IrReceiver.decodedIRData.command == IR_ADVANCE) {
        Serial.println("avant");
        state = ADVANCE;
      } else if (IrReceiver.decodedIRData.command == IR_RIGHT) {
        Serial.println("droite");
        state = RIGHT;
      } else if (IrReceiver.decodedIRData.command == IR_LEFT) {
        Serial.println("gauche");
        state = LEFT;
      } else if (IrReceiver.decodedIRData.command == IR_BACK) {
        Serial.println("arrière");
        state = BACK;
      } else if (IrReceiver.decodedIRData.command == IR_STOP) {
        Serial.println("stop");
        state = STOP;
      } else if (IrReceiver.decodedIRData.command == IR_TANKLEFT) {
        Serial.println("tankleft");
        state = TANKLEFT;
      } else if (IrReceiver.decodedIRData.command == IR_TANKRIGHT) {
        Serial.println("tankright");
        state = TANKRIGHT;
      } else if (IrReceiver.decodedIRData.command == IR_TWERK) {
        Serial.println("TWERK");
        state = TWERK;
      } else if (IrReceiver.decodedIRData.command == IR_KLAXON) {
        Serial.println("KLAXON");
        state = KLAXON;
      } else if (IrReceiver.decodedIRData.command == IR_AUTO) {
        Serial.println("AUTO");
        state = AUTO;
      }
    } else {
      //IrReceiver.printIRResultRawFormatted(&Serial, true);
    }
  }
  IrReceiver.resume();
}

void move(int speedL, int speedR, bool FR, bool RR, bool FL, bool RL) {
  analogWrite(speedPinL, speedL);
  analogWrite(speedPinR, speedR);
  digitalWrite(RightDirectPin1, FR ? HIGH : LOW);
  digitalWrite(RightDirectPin2, RR ? HIGH : LOW);
  digitalWrite(LeftDirectPin1, FL ? HIGH : LOW);
  digitalWrite(LeftDirectPin2, RL ? HIGH : LOW);
}

float checkdistance() {
  digitalWrite(UltrasonicTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(UltrasonicTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(UltrasonicTrigPin, LOW);
  float distance = pulseIn(UltrasonicEchoPin, HIGH) * 0.034 / 2;
  delay(10);
  return distance;
}

float checkdistanceAround() {
  myservo.write(10);
  delay(200);
  int distance1 = checkdistance();
  myservo.write(55);
  delay(200);
  int distance2 = checkdistance();
  myservo.write(90);
  delay(200);
  int distance3 = checkdistance();
  myservo.write(55);
  delay(200);
  return min(min(distance1, distance2), distance3);
}

DIRECTION checkDirection() {
  int distance[3] = { 0, 0, 0 };
  myservo.write(0);
  delay(350);
  distance[0] = checkdistance();
  myservo.write(55);
  delay(350);
  distance[1] = checkdistance();
  myservo.write(120);
  delay(350);
  distance[2] = checkdistance();
  myservo.write(55);
  delay(50);

  if (min(min(distance[0], distance[1]), distance[2]) > 25) {
    return CONTINUE;
  }

  int min = 10000;
  int minindex = 0;
  for (int i = 0; i < 3; i++) {
    if (min > distance[i]) {
      min = distance[i];
      minindex = i;
    }
  }
  switch (minindex) {
    case 0:
      return GOLEFT;
      break;
    case 1:
      return GOFRONT;
      break;
    case 2:
      return GORIGHT;
      break;
  }
}

void autoTick() {
  myservo.write(55);

  DIRECTION dir = checkDirection();
  Serial.println(dir);
  if (dir == CONTINUE) {
    if (random(0, 8) != 1) {
      move(511, 511, true, false, true, false);
      delay(300);
      move(0, 0, false, false, false, false);
      delay(200);
    } else {
      if (random(0, 2) != 1) {
        move(511, 511, false, true, true, false);
        delay(300);
        move(0, 0, false, false, false, false);
        delay(200);
      } else {
        move(511, 511, true, false, false, true);
        delay(300);
        move(0, 0, false, false, false, false);
        delay(200);
      }
    }
  } else {
    if (random(0, 4) != 1) {
      move(511, 511, false, true, true, false);
      delay(550);
      move(0, 0, false, false, false, false);
      delay(300);
    } else {
      if (dir == GOLEFT || dir == GOFRONT) {
        move(511, 511, true, false, false, true);
        delay(150);
        move(0, 0, false, false, false, false);
        delay(200);
      } else if (dir == GORIGHT) {
        move(511, 511, false, true, true, false);
        delay(150);
        move(0, 0, false, false, false, false);
        delay(200);
      }
    }
  }
}

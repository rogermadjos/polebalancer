#include "fuzzy.h"

#define PIN_SENSOR        0
#define PIN_MOTORA_DIR    8
#define PIN_MOTORA_CTRL   9
#define PIN_MOTORB_DIR    10
#define PIN_MOTORB_CTRL   11
#define MOTORA            34
#define MOTORB            87

float desiredAngle = 0;

void setMotor(byte motor, float spd) {
  int dirPin = (motor == MOTORA) ? PIN_MOTORA_DIR : PIN_MOTORB_DIR;
  int ctrlPin = (motor == MOTORA) ? PIN_MOTORA_CTRL : PIN_MOTORB_CTRL;
  int dir = (spd < 0) ? LOW : HIGH;
  int pwm = min(abs((int)spd),255);
  digitalWrite(dirPin,dir);
  analogWrite(ctrlPin,pwm);
}

void setup() {
  
  pinMode(PIN_MOTORA_DIR,OUTPUT);
  pinMode(PIN_MOTORA_CTRL,OUTPUT);
  pinMode(PIN_MOTORB_DIR,OUTPUT);
  pinMode(PIN_MOTORB_CTRL,OUTPUT);
  
}

void loop() {
  float actual = analogRead(PIN_SENSOR);
  float ctrl = control(desiredAngle, actual);
  setMotor(MOTORA, ctrl);
  setMotor(MOTORB, ctrl);
  delay(5);
}

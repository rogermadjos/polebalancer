#include "control.h"
#include <Arduino.h> 

#define SATURATION 30
#define ALPHA 0.75f
#define Kp  25
#define Ki  0
#define Kd  0

boolean start = true;

float lError = 0;
float iError = 0;
float dError = 0;

long lastReadTime = 0;

float control(float desired, float actual) {
  long readTime = micros();
  float h = (readTime - lastReadTime)/1000000.0f;
  float P = desired - actual;
  float D = (P - lError)/h;
  D = (1-ALPHA) * dError + ALPHA * D;
  iError += P*h;
  iError = (iError > SATURATION) ? SATURATION : iError;
  iError = (iError < -SATURATION) ? -SATURATION : iError;
  lError = P;
  dError = D;
  lastReadTime = readTime;
  float I = iError;
  if(start) {
    D = 0;
    I = 0;
    dError = 0;
    start = false;
  }
  return P * Kp + I * Ki + D * Kd;
}

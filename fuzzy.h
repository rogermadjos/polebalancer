#include "control.h"
#include <Arduino.h> 

#define NUM_ANTECEDENTS 2

#define INF 1000000.0f
#define LIM  1.0f
#define ORIG 0.0f
#define ALPHA 0.75f

#define Kp  0.2f
#define Kd  0.5f
#define Ko  50.0f

const float pFuzzyCenters[] =    { ORIG , 0.0625f , 0.2500f , 0.5625f ,    LIM };
const float dFuzzyCenters[] =    { ORIG , 0.3333f , 0.6667f ,     LIM };
float fuzzyAntecedents[] = {-LIM , -0.7500f , -0.5000f , -0.2500f, ORIG , 0.2500f , 0.5000f , 0.7500f , LIM };

const int fuzzyRules[][9] = {
{  4,  4,  4,  4,  3,  2,  1,  0, -1},
{  4,  4,  4,  3,  2,  1,  0, -1, -2},
{  4,  4,  3,  2,  1,  0, -1, -2, -3},
{  4,  3,  2,  1,  0, -1, -2, -3, -4},
{  3,  2,  1,  0, -1, -2, -3, -4, -4},
{  2,  1,  0, -1, -2, -3, -4, -4, -4},
{  1,  0, -1, -2, -3, -4, -4, -4, -4}
};

boolean start = true;
long lastReadTime = 0;
float lError = 0;
float dError = 0;

float fuzzify(float l, float c, float r, float v) {
  if((l == -INF && v < c) || (r == INF && v > c))
    return 1;
  if(v < l || v > r) 
    return 0;
  if(v >= l && v <= c)
    return (v - l) / (c - l);
  if(v <= r && v > c)
    return (r - v) / (r - c);
}

float fuzzify(const float * fuzzyC, int mf, float v) {
  float l,r,c;
  if(mf > 0) {
    c = fuzzyC[mf];
    if(c == LIM)
      r = INF;
    else
      r = fuzzyC[mf+1];
    l = fuzzyC[mf-1];
  }
  else if(mf < 0) {
    c = -fuzzyC[abs(mf)];
    if(c == -LIM)
      l = -INF;
    else
      l = -fuzzyC[abs(mf)+1];
    r = -fuzzyC[abs(mf)-1];
  }
  else if(mf == 0) {
    c = 0;
    l = -fuzzyC[1];
    l = fuzzyC[1];
  }
  return fuzzify(l,c,r,v);
}

float control(float desired, float actual) {
  long readTime = micros();
  float h = (readTime - lastReadTime)/1000000.0f;
  float P = desired - actual;
  float D = (P - lError)/h;
  D = (1-ALPHA) * dError + ALPHA * D;
  lError = P;
  dError = D;
  lastReadTime = readTime;
  if(start) {
    D = 0;
    dError = 0;
    start = false;
  }
  //fuzzy inference;
  float tw = 0;
  float fuzzyV = 0;
  for(int e = -4; e <= 4; e++) {
    for(int edot = -3; edot <= 3; edot++) {
      int consequent = fuzzyRules[edot + 3][e + 4];
      float v = min(fuzzify(pFuzzyCenters,e,P),fuzzify(dFuzzyCenters,edot,D));
      tw += v;
      fuzzyV += v * fuzzyAntecedents[consequent + 4];
    }
  }
  return fuzzyV / tw;
}




#include <Arduino.h>
#include "enc.h"
#include <LibPrintf.h>
#include <Encoder.h>

int prevPos = 0;

Encoder myenc(ENC_CHANNEL_A, ENC_CHANNEL_B);

void setup() {
  // pinMode(ENC_CHANNEL_A, INPUT);
  // pinMode(ENC_CHANNEL_B, INPUT);
  // pinMode(ENC_CHANNEL_INDX, INPUT);

  Serial.begin(9600);

  // int prevA = digitalRead(ENC_CHANNEL_A);
}

void loop() {

  double newPos, revolutions;

  newPos = myenc.read();

  if(newPos != prevPos){
    revolutions = newPos/(2048.0*4.0);
    printf("Count: %f\t Revs: %3.1f\n", newPos, revolutions);
    
    prevPos = newPos;
  }
  
}
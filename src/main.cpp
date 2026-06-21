#include <Arduino.h>
#include "enc.h"
#include <LibPrintf.h>
#include <Encoder.h>
#include "stepper.h"

int prevPos = 0;

Encoder myenc(ENC_CHANNEL_A, ENC_CHANNEL_B);
stepper linearStepper(5,6);

void setup() {

  Serial.begin(9600);

  linearStepper.enable();

}

void loop() {

  double newPos, revolutions;

  newPos = myenc.read();

  if(newPos != prevPos){
    revolutions = newPos/(2048.0*4.0);
    printf("Count: %f\t Revs: %3.1f\n", newPos, revolutions);
    
    prevPos = newPos;
  }

  int steps;

  if(newPos > prevPos){
    //going CW?

  }

  // linearStepper.step(1, );
  
}
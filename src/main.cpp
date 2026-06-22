#include <Arduino.h>
#include "enc.h"
#include <LibPrintf.h>
#include <Encoder.h>
#include "stepper.h"

int prevPos = 0;

#define ROTATIONS_PER_LAYER 23
#define WIRE_D 0.43
#define LINEAR_RES 0.01

Encoder myenc(ENC_CHANNEL_A, ENC_CHANNEL_B);
stepper linearStepper(5,6);

void setup() {

  Serial.begin(9600);

  linearStepper.enable();

}

void loop() {

  double newPos, revolutions;

  newPos = myenc.read();

  //Serial Monitor
  if(newPos != prevPos){
    revolutions = newPos/(RESOLUTION*4.0);
    printf("Count: %f\t Revs: %3.1f\n", newPos, revolutions);
    
    prevPos = newPos;
  }

  int steps;

  if(newPos > prevPos){
    //going CW?

    steps = (newPos - prevPos) * ((RESOLUTION*4) / ((ROTATIONS_PER_LAYER * WIRE_D) / LINEAR_RES));

    linearStepper.step(1,steps);
  }
  else{
    //going CCW?
    steps = abs(newPos - prevPos) * ((RESOLUTION*4) / ((ROTATIONS_PER_LAYER * WIRE_D) / LINEAR_RES));

    linearStepper.step(0,steps);
  }

  // linearStepper.step(1, );
  
}
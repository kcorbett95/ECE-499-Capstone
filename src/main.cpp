#include <Arduino.h>
#include "enc.h"
#include <LibPrintf.h>
#include <Encoder.h>
#include "stepper.h"

long prevPos = 0;
long issuedSteps = 0;

#define ROTATIONS_PER_LAYER 23
#define WIRE_D 0.43
#define LINEAR_RES 0.00106

#define STEP_PIN 9
#define DIR_PIN 10

const double STEPS_PER_COUNT = (WIRE_D / LINEAR_RES) / (RESOLUTION * 4.0);


Encoder myenc(ENC_CHANNEL_A, ENC_CHANNEL_B);
stepper linearStepper(STEP_PIN,DIR_PIN);

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

  long targetSteps = lround(newPos * STEPS_PER_COUNT);
  long deltaSteps = targetSteps - issuedSteps;

  if(deltaSteps > 0){
    //going CW?

    linearStepper.step(1, deltaSteps);
    issuedSteps += deltaSteps;
  }
  else if(deltaSteps < 0){
    //going CCW?

    linearStepper.step(0, -deltaSteps);
    issuedSteps += deltaSteps;
  }
  
}
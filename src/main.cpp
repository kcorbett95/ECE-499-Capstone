#include <Arduino.h>
#include <LibPrintf.h>
#include <Encoder.h>
#include "stepper.h"

/*========== ENCODER =========*/
/*
/   Encoder Model: AMT102-V
/   Pinout:
/       Blue: B-Channel
/       Red: +5V
/       Green: A-Channel
/       Purple: Index Channel
/       Black: GND
/   DIP Settings: 0 0 0 0
/   Resolution (PPR): 2048/Rotation
/   
*/

#define ENC_CHANNEL_A 2
#define ENC_CHANNEL_B 3
#define ENC_CHANNEL_INDX 4
#define RESOLUTION 2048
/*=============================*/

/*========== MISC DEFINES ==========*/
#define ROTATIONS_PER_LAYER 23 //
#define WIRE_DIAMETER 0.43 //0.43mm
#define LINEAR_RES 0.00106 //Verified through measurements
#define STEP_PIN 9
#define DIR_PIN 10
/*=============================*/

/*========== GLOBALS ==========*/
int prevPos = 0;
long issuedSteps = 0;
const double STEPS_PER_COUNT = (WIRE_DIAMETER / LINEAR_RES) / (RESOLUTION * 4.0);
/*=============================*/

/*========== CLASSES ==========*/
stepper linearStepper(STEP_PIN,DIR_PIN);
Encoder myenc(ENC_CHANNEL_A, ENC_CHANNEL_B);
/*=============================*/


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
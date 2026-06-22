#include <Arduino.h>
#include <LibPrintf.h>
#include <Encoder.h>
#include <TMCStepper.h>




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

Encoder myenc(ENC_CHANNEL_A, ENC_CHANNEL_B);
/*=============================*/

/*========== GLOBALS ==========*/
int prevPos = 0;
/*=============================*/

void setup() {

  Serial.begin(9600);

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
#include <Arduino.h>
#include <LibPrintf.h>
#include <Encoder.h>
#include "stepper.h"
#include <LiquidCrystal.h>

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
#define STEP_PIN 8
#define DIR_PIN 9
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

// LiquidCrystal(rs, en, d4, d5, d6, d7)
LiquidCrystal lcd(12, 11, 4, 5, 6, 7);
void setup() {

    delay(100);         // allow LCD to settle after power-on
    lcd.begin(16, 2);   // 16 columns, 2 rows
    lcd.clear();

    lcd.setCursor(0, 0);
    lcd.print("ViVitro Labs");

    lcd.setCursor(0, 1);
    lcd.print("Capstone 2026");

    delay(2000);

    lcd.setCursor(0, 0);
    lcd.print("N_Turns:             ");

    lcd.setCursor(0, 1);
    lcd.print("Count:               ");

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

    char buf[9];
    lcd.setCursor(8, 0);
    dtostrf(revolutions, 7, 2, buf);
    lcd.print(buf);

    lcd.setCursor(7, 1);
    dtostrf(newPos, 8, 0, buf);
    lcd.print(buf);
    
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
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
/*
 * =============================================================================
 * ARDUINO NANO PIN CONNECTION TABLE
 * =============================================================================
 *
 * LCD 16x2 (HD44780, 4-bit mode)
 * ---------------------------------------------------------------------------
 *  LCD Pin    Signal           Nano Pin    Notes
 *  -------    ------           --------    -----
 *  VSS        GND              GND
 *  VDD        +5V              5V
 *  RS         Register Sel.    D12
 *  RW         Read/Write       GND         Write-only mode
 *  EN         Enable           D11
 *  D4         Data 4           D4
 *  D5         Data 5           D5
 *  D6         Data 6           D6
 *  D7         Data 7           D7
 *  A  (BL+)   Backlight +      5V          Via 220 ohm series resistor
 *  K  (BL-)   Backlight -      GND
 *
 * Encoder: AMT102-V
 * ---------------------------------------------------------------------------
 *  Wire       Signal           Nano Pin    Notes
 *  ----       ------           --------    -----
 *  Red        +5V              5V
 *  Black      GND              GND
 *  Green      A-Channel        D2          INT0
 *  Blue       B-Channel        D3          INT1
 *  Purple     Index            NC          Unused
 *
 * TMC2209 Stepper Driver #1 (Linear Axis)
 * ---------------------------------------------------------------------------
 *  Signal     Nano Pin         Notes
 *  ------     --------         -----
 *  STEP       D8
 *  DIR        D9
 *  EN         GND              Tied low = always enabled
 *  CLK        GND              Tied low = internal clock
 *  VDD        5V               Logic supply
 *  VM         24V              Motor power supply
 *  GND        GND              Common with 24V supply GND
 *
 * TMC2209 Stepper Driver #2
 * ---------------------------------------------------------------------------
 *  Signal     Nano Pin         Notes
 *  ------     --------         -----
 *  STEP       D10
 *  DIR        D13              Shares onboard LED
 *  EN         GND              Tied low = always enabled
 *  CLK        GND              Tied low = internal clock
 *  VDD        5V               Logic supply
 *  VM         24V              Motor power supply
 *  GND        GND              Common with 24V supply GND
 *
 * End-Stop Switch (NO, 3-wire, 24V)
 * ---------------------------------------------------------------------------
 *  Wire       Signal           Connection  Notes
 *  ----       ------           ----------  -----
 *  Brown      +24V             24V supply
 *  Blue       0V / GND         GND         Common with Arduino GND
 *  Black      NO contact       A2          INPUT_PULLUP; LOW = tripped
 *
 * Jog Buttons (momentary NO, one side to GND)
 * ---------------------------------------------------------------------------
 *  Button     Nano Pin         Notes
 *  ------     --------         -----
 *  Jog CW     A0               INPUT_PULLUP; LOW = pressed
 *  Jog CCW    A1               INPUT_PULLUP; LOW = pressed
 *
 * Reset Button
 * ---------------------------------------------------------------------------
 *  RESET pin to GND            Internal pull-up; no I/O pin required
 *
 * =============================================================================
 */

int prevPos = 0;
long issuedSteps = 0;
const double STEPS_PER_COUNT = (WIRE_DIAMETER / LINEAR_RES) / (RESOLUTION * 4.0);
/*=============================*/

/*========== CLASSES ==========*/
stepper linearStepper(STEP_PIN,DIR_PIN);
Encoder myenc(ENC_CHANNEL_A, ENC_CHANNEL_B);

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

  revolutions = newPos/(RESOLUTION*4.0);

  //Serial Monitor
  if(newPos != prevPos){
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
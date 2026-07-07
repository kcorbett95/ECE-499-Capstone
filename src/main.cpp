#include <Arduino.h>
#include <LibPrintf.h>
#include <Encoder.h>
#include "stepper.h"
#include <LiquidCrystal.h>
#include <AceButton.h>
#include <math.h>

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
 * TMC2209 Stepper Driver #2 (Rotary Axis)
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
 * End-Stop Switch: FPS-1A-NPN-NO (photoelectric, 3-wire, 24V)
 * ---------------------------------------------------------------------------
 *  NPN output sinks to 0V when active, floats when inactive.
 *  Safe for direct Arduino connection with INPUT_PULLUP.
 *  Arduino GND and 24V supply GND must share a common reference.
 *
 *  Wire       Signal           Connection  Notes
 *  ----       ------           ----------  -----
 *  Brown      +24V             24V supply
 *  Blue       0V / GND         GND         Common with Arduino GND
 *  Black      NPN output       A3          INPUT_PULLUP; LOW = tripped
 *
 * Jog Buttons (momentary NO, one side to GND)
 * ---------------------------------------------------------------------------
 *  Button     Nano Pin         Notes
 *  ------     --------         -----
 *  Jog CW     A0               INPUT_PULLUP; LOW = pressed
 *  Jog CCW    A2               INPUT_PULLUP; LOW = pressed
 *
 * Reset Button
 * ---------------------------------------------------------------------------
 *  RESET pin to GND            Internal pull-up; no I/O pin required
 *
 * =============================================================================
 */

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
*/
#define ENC_CHANNEL_A   2
#define ENC_CHANNEL_B   3
#define ENC_CHANNEL_INDX 4
#define RESOLUTION      2048
/*=============================*/

/*========== MISC DEFINES ==========*/
#define ROTATIONS_PER_LAYER 23
#define WIRE_DIAMETER   0.43    // mm
#define LINEAR_RES      0.00106 // mm per microstep, verified by measurement
#define LINEAR_STEP_PIN        8
#define LINEAR_DIR_PIN         9
#define ROTARY_STEP_PIN        10
#define ROTARY_DIR_PIN         13
#define ENDSTOP_PIN     A3
#define HOME_DIR        0       // right to left, toward the endstop
#define START_POS_MM    5.3    // mm from endstop to winding start position
#define START_RIGHT_SHIFT_MM 6  //Starting shift to the right
#define JOG_RIGHT_BTN_PIN   A2
#define JOG_LEFT_BTN_PIN   A0
#define LEAD_LAG_COMP 0.3       //Half of a rotation buffer at each end
/*=============================*/

/*========== GLOBALS ==========*/
long prevPos = 0;
long issuedSteps = 0;
long targetSteps = 0;
bool atEndFlag = false;
double endFlagTriggerRevs = 0;
// Steps for one full traversal pass across the layer width
const long LAYER_STEPS = lround((double)ROTATIONS_PER_LAYER * WIRE_DIAMETER / LINEAR_RES);
// const long LINEAR_DIST_COMP = lround((WIRE_DIAMETER / LINEAR_RES) * LEAD_LAG_COMP);
/*=============================*/

/*========== OBJECTS ==========*/
stepper linearStepper(LINEAR_STEP_PIN, LINEAR_DIR_PIN);
// stepper rotaryStepper(ROTARY_DIR_PIN, ROTARY_STEP_PIN);
Encoder myenc(ENC_CHANNEL_A, ENC_CHANNEL_B);
// LiquidCrystal(rs, en, d4, d5, d6, d7)
LiquidCrystal lcd(12, 11, 4, 5, 6, 7);
/*=============================*/

// Drives linearStepper toward the endstop, backs off until the switch
// releases, moves to START_POS_MM, then zeros all position tracking so
// encoder=0 corresponds to the winding start position.
// Halts with an LCD error if the endstop is not reached within MAX_HOME_STEPS.
void home() {
    const long MAX_HOME_STEPS = 500000;
    long steps = 0;

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Homing...");

    //Move carriage to the right by 6mm to ensure it doesnt hit the left most limit
    linearStepper.step(1, (int)(START_POS_MM / LINEAR_RES));

    while (digitalRead(ENDSTOP_PIN) == HIGH && steps < MAX_HOME_STEPS) {
        linearStepper.step(HOME_DIR, 1);
        steps++;
    }

    if (steps >= MAX_HOME_STEPS) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("HOMING FAILED");
        while (true);
    }

    // Back off until switch releases
    while (digitalRead(ENDSTOP_PIN) == LOW) {
        linearStepper.step(!HOME_DIR, 1);
    }

    // Move to winding start position, then zero tracking
    long startSteps = lround(START_POS_MM / LINEAR_RES);
    linearStepper.step(!HOME_DIR, startSteps);

    issuedSteps = 0;
    myenc.write(0);
    prevPos = 0;
}

void setup() {

    delay(100);
    lcd.begin(16, 2);
    lcd.clear();

    lcd.setCursor(0, 0);
    lcd.print("ViVitro Labs");
    lcd.setCursor(0, 1);
    lcd.print("Capstone 2026");
    delay(2000);

    Serial.begin(9600);
    linearStepper.enable();

    pinMode(ENDSTOP_PIN, INPUT_PULLUP);
    home(); //Homes the linear drive carrage

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("N_Turns:             ");
    lcd.setCursor(0, 1);
    lcd.print("Count:               ");
}

void jogBtn(int direction, int num_steps) {
    linearStepper.step(direction, num_steps);
    issuedSteps += (direction == 1) ? num_steps : -num_steps;
}

void loop() {

    long newPos = myenc.read();
    double revolutionsTotal = (double)newPos / (RESOLUTION * 4.0);
    int passNumber = floor((int)(revolutionsTotal / ROTATIONS_PER_LAYER));  //Rounds Down Always
    double revolutionsInPass = revolutionsTotal - (passNumber * ROTATIONS_PER_LAYER);

    if (newPos != prevPos) {
        char buf[9];
        lcd.setCursor(8, 0);
        dtostrf(revolutionsTotal, 7, 2, buf);
        lcd.print(buf);

        lcd.setCursor(7, 1);
        dtostrf((double)newPos, 8, 0, buf);
        lcd.print(buf);

        printf("Count: %ld\t Pass: %ld\t Turns: %.2f\n", newPos, passNumber, revolutionsTotal);

        prevPos = newPos;
    }

    // Linear steps completed within the current pass
    long stepsInPass = lround(revolutionsInPass * WIRE_DIAMETER / LINEAR_RES);

    // At each pass boundary, pause linear motion for LEAD_LAG_COMP rotations.
    // Triggered by entering the first LEAD_LAG_COMP rotations of any new pass (passNumber > 0).
    // endFlagTriggerRevs stores the exact revolution count at the boundary so the
    // resume condition is evaluated against a fixed reference, not a moving target.
    if (!atEndFlag && passNumber > 0 && revolutionsInPass < LEAD_LAG_COMP) {
        atEndFlag = true;
        endFlagTriggerRevs = passNumber * ROTATIONS_PER_LAYER;
    }

    // Resume once LEAD_LAG_COMP rotations have elapsed past the boundary.
    // Sync issuedSteps to the current encoder position so deltaSteps = 0
    // at the moment of resumption — no catch-up jump.
    if (atEndFlag && revolutionsTotal >= endFlagTriggerRevs + LEAD_LAG_COMP) {
        atEndFlag = false;
        if (passNumber % 2 == 0) {
            issuedSteps = stepsInPass;
        } else {
            issuedSteps = LAYER_STEPS - stepsInPass;
        }
    }

    if (!atEndFlag) {
        // Even pass: traverse forward; odd pass: traverse in reverse
        if (passNumber % 2 == 0) {
            targetSteps = stepsInPass;
        } else {
            targetSteps = (LAYER_STEPS - stepsInPass);
        }

        // Issue only the new steps needed to reach targetSteps
        long deltaSteps = targetSteps - issuedSteps;
        if (deltaSteps > 0) {
            linearStepper.step(1, deltaSteps);
            issuedSteps += deltaSteps;
        } else if (deltaSteps < 0) {
            linearStepper.step(0, -deltaSteps);
            issuedSteps += deltaSteps;
        }
    }
}

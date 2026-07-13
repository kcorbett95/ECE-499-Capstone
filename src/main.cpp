#include <Arduino.h>
#include <LibPrintf.h>
#include <Encoder.h>
#include "stepper.h"
#include <LiquidCrystal.h>
#include <AceButton.h>
#include <math.h>
#include <AccelStepper.h>

using namespace ace_button;

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
#define WIRE_DIAMETER   0.41    // mm
#define LINEAR_RES      0.00106 // mm per microstep, verified by measurement
#define LINEAR_STEP_PIN        8
#define LINEAR_DIR_PIN         9
#define ROTARY_STEP_PIN        10
#define ROTARY_DIR_PIN         13
#define ENDSTOP_PIN     A3
#define HOME_DIR        0       // right to left, toward the endstop
#define START_POS_MM    7.8    // mm from endstop to winding start position
#define START_RIGHT_SHIFT_MM 6  //Starting shift to the right
#define JOG_RIGHT_BTN_PIN   A2  // A1 pin is broken on the Nano board
#define JOG_LEFT_BTN_PIN    A0
#define JOG_MM              0.5      // mm to move per button press
#define END_JOG_MM          .86     // mm to retract from bobbin end when pass boundary is hit
#define LEAD_LAG_COMP       0.43     //Half of a rotation buffer at each end
#define LINEAR_MAX_SPEED      10000     //Steps per second
#define LINEAR_ACCELERATION     5000      //Steps per second per second
#define ROTARY_MAX_SPEED        2000     //Steps per second, tune to desired winding RPM
#define ROTARY_ACCELERATION     2000     //Steps per second per second, soft start/stop ramp
#define ROTARY_RUN_DIRECTION       1     //1 or -1, tune to match desired winding direction
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

// Rotary drive state: IDLE (not moving) -> RUNNING (spinning, button-started) ->
// STOPPING (decelerating) -> back to IDLE once fully stopped.
enum WindingState { WIND_IDLE, WIND_RUNNING, WIND_STOPPING };
WindingState windingState = WIND_IDLE;
/*=============================*/

/*========== OBJECTS ==========*/
// stepper linearStepper(LINEAR_STEP_PIN, LINEAR_DIR_PIN);  //OLD DEFINITION
AccelStepper LinearStepper(AccelStepper::DRIVER, LINEAR_STEP_PIN, LINEAR_DIR_PIN);
// stepper rotaryStepper(ROTARY_DIR_PIN, ROTARY_STEP_PIN);
AccelStepper RotaryStepper(AccelStepper::DRIVER, ROTARY_STEP_PIN, ROTARY_DIR_PIN);
Encoder myenc(ENC_CHANNEL_A, ENC_CHANNEL_B);
// LiquidCrystal(rs, en, d4, d5, d6, d7)
LiquidCrystal lcd(12, 11, 4, 5, 6, 7);
AceButton jogRightBtn(JOG_RIGHT_BTN_PIN);
AceButton jogLeftBtn(JOG_LEFT_BTN_PIN);
/*=============================*/

// Services RotaryStepper without blocking. Must be called every loop()
// iteration, and inside any blocking while-loop elsewhere (home(), jogBtn())
// so a homing pass or a jog never stalls an in-progress winding rotation.
void pumpRotary() {
    if (windingState == WIND_RUNNING) {
        // Continuously re-arm a far-off target so RotaryStepper ramps up via
        // its normal accel profile and then cruises indefinitely, instead of
        // jumping straight to speed.
        if (RotaryStepper.distanceToGo() == 0) {
            RotaryStepper.move(ROTARY_RUN_DIRECTION * 2000000000L);
        }
        RotaryStepper.run();
    } else if (windingState == WIND_STOPPING) {
        RotaryStepper.run();
        if (!RotaryStepper.isRunning()) {
            windingState = WIND_IDLE;
        }
    }
}

// Toggles the rotary winding drive on/off. Called from the (repurposed)
// right jog button. Starting ramps up smoothly; stopping decelerates under
// AccelStepper's own ramp rather than an instant halt.
void toggleWinding() {
    if (windingState == WIND_IDLE) {
        RotaryStepper.setCurrentPosition(0);
        RotaryStepper.move(ROTARY_RUN_DIRECTION * 2000000000L);
        windingState = WIND_RUNNING;
    } else if (windingState == WIND_RUNNING) {
        RotaryStepper.stop();
        windingState = WIND_STOPPING;
    }
    // WIND_STOPPING: ignore extra presses, let the in-flight stop finish.
}

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

    //Move carriage to the right until it is outside the beam
    while(digitalRead(ENDSTOP_PIN) == LOW){
        LinearStepper.move(1000);
        LinearStepper.run();
        pumpRotary();
    }
    LinearStepper.stop();

    //Move carriage to the left until it is outside the beam
    while(digitalRead(ENDSTOP_PIN) == HIGH){
        LinearStepper.move(-1000);
        LinearStepper.run();
        pumpRotary();
    }
    LinearStepper.stop();   //Stop once home has bit reached
    LinearStepper.setCurrentPosition(0);    //Temporary home position

    if (steps >= MAX_HOME_STEPS) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("HOMING FAILED");
        while (true);
    }

    // Move to winding start position, then zero tracking
    long startPositionSteps = lround(START_POS_MM / LINEAR_RES);
    LinearStepper.runToNewPosition(startPositionSteps);

    issuedSteps = 0;
    myenc.write(0);
    prevPos = 0;
    LinearStepper.setCurrentPosition(0);    //Sets current position to position 0. Final Reference point.
}

void jogBtn(int direction, int num_steps) {
    // linearStepper.step(direction, num_steps);    //! Old Implementation
    int currentPosTemp = LinearStepper.currentPosition();
    LinearStepper.move(direction * num_steps);
    LinearStepper.setAcceleration(LINEAR_ACCELERATION*4);
    while(currentPosTemp != LinearStepper.currentPosition() + num_steps){
        LinearStepper.run();
        pumpRotary();
        printf("Stepper Stepping - BTN Press\n");
    }
    LinearStepper.stop();
    printf("Stepper Stopped - BTN Press\n");
    LinearStepper.setAcceleration(LINEAR_ACCELERATION);
    // Reset issuedSteps to targetSteps so deltaSteps = 0 on the next loop
    // iteration. The winding loop then sees no error and issues no correction
    // steps — the motor holds its new jogged position.
    issuedSteps = targetSteps;
}

void handleJogEvent(AceButton* button, uint8_t eventType, uint8_t /*buttonState*/) {
    if (eventType != AceButton::kEventPressed) return;
    if (button->getPin() == JOG_RIGHT_BTN_PIN) {
        toggleWinding();
    } else if (button->getPin() == JOG_LEFT_BTN_PIN) {
        // TODO: slated to become reset-turn-count + re-home in a future pass.
        jogBtn(-1, lround(JOG_MM / LINEAR_RES));
    }
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

    LinearStepper.setMaxSpeed(LINEAR_MAX_SPEED);
    LinearStepper.setAcceleration(LINEAR_ACCELERATION);
    RotaryStepper.setMaxSpeed(ROTARY_MAX_SPEED);
    RotaryStepper.setAcceleration(ROTARY_ACCELERATION);

    pinMode(ENDSTOP_PIN, INPUT_PULLUP);
    pinMode(JOG_RIGHT_BTN_PIN, INPUT_PULLUP);
    pinMode(JOG_LEFT_BTN_PIN, INPUT_PULLUP);
    ButtonConfig::getSystemButtonConfig()->setEventHandler(handleJogEvent);

    home(); //Homes the linear drive carrage

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("N_Turns:             ");
    lcd.setCursor(0, 1);
    lcd.print("Count:               ");
}

void loop() {

    jogRightBtn.check();
    jogLeftBtn.check();

    pumpRotary();
    LinearStepper.run();

    long newPos = myenc.read();
    double revolutionsTotal = (double)newPos / (RESOLUTION * 4.0);
    int passNumber = (int)(revolutionsTotal / ROTATIONS_PER_LAYER);  //Rounds Down Always
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
        // Retract from the bobbin end before waiting for LEAD_LAG_COMP turns.
        // Direction is opposite to the pass just completed:
        //   even pass = forward (dir 1), so retract backward (dir 0)
        //   odd pass  = backward (dir 0), so retract forward (dir 1)
        int retractDir = (passNumber % 2 == 0) ? -1 : 1;
        // linearStepper.step(retractDir, lround(END_JOG_MM / LINEAR_RES));
        LinearStepper.move(retractDir * lround(END_JOG_MM / LINEAR_RES));
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
            // linearStepper.step(1, deltaSteps);
            issuedSteps += deltaSteps;
        } else if (deltaSteps < 0) {
            // linearStepper.step(0, -deltaSteps);
            issuedSteps += deltaSteps;
        }
    }
}

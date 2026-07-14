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
 *  EN         A4               LOW = enabled/holding, HIGH = disabled/free-spinning
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
 * Control Buttons (momentary NO, one side to GND)
 * ---------------------------------------------------------------------------
 *  Button                  Nano Pin         Notes
 *  ------                  --------         -----
 *  Home (Right)            A0               INPUT_PULLUP; LOW = pressed
 *  Start/Stop (Left)       A2               INPUT_PULLUP; LOW = pressed
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
#define WIRE_DIAMETER   0.39    // mm
#define LINEAR_RES      0.00106 // mm per microstep, verified by measurement
#define LINEAR_STEP_PIN        8
#define LINEAR_DIR_PIN         9
#define ROTARY_STEP_PIN        10
#define ROTARY_DIR_PIN         13
#define ROTARY_EN_PIN           A4     // TMC2209 #2 EN: LOW = enabled/holding, HIGH = disabled/free-spinning
#define ENDSTOP_PIN     A3
#define HOME_DIR        0       // right to left, toward the endstop
#define START_POS_MM    7.8    // mm from endstop to winding start position
#define START_RIGHT_SHIFT_MM 6  //Starting shift to the right
#define START_STOP_BTN_PIN   A2  // A1 pin is broken on the Nano board
#define HOME_BTN_PIN    A0
#define END_JOG_MM          0.8     // mm to retract from bobbin end when pass boundary is hit
#define LINEAR_MAX_SPEED      20000     //Steps per second
#define LINEAR_ACCELERATION     10000      //Steps per second per second
#define ROTARY_MAX_SPEED        4000     //Steps per second, tune to desired winding RPM
#define ROTARY_ACCELERATION     2000     //Steps per second per second, soft start ramp
#define ROTARY_START_SPEED        200.0   //Steps per second, ramp starting point
#define ROTARY_RUN_DIRECTION       1     //1 or -1, tune to match desired winding direction
#define TURNS_PER_BOBBIN          92.5   //Encoder turns to wind on EACH bobbin before advancing phase
#define SLACK_TURNS                3.0   //Turns of slack wire wound in the gap between the two bobbins
// PLACEHOLDERS — measure on the bench and tune. GAP_POS_MM is where the
// carriage parks while the slack turns are wound; BOBBIN_B_START_POS_MM is
// bobbin B's own zero reference (mirrors START_POS_MM/START_RIGHT_SHIFT_MM
// for bobbin A), both measured from the same endstop-homed zero as bobbin A.
#define GAP_POS_MM                20.0
#define BOBBIN_B_START_POS_MM     30.0
// Bobbin B is mounted the opposite way around, so its first pass needs to
// travel away from the gap just like bobbin A's does — flip this if the
// carriage starts bobbin B by moving the wrong direction on the bench.
#define BOBBIN_B_MIRRORED         true
/*=============================*/

/*========== GLOBALS ==========*/
long prevPos = 0;
long targetSteps = 0;
bool atEndFlag = false;
// Which pass boundary has already triggered a retract, so re-entering the
// same boundary can't re-trigger it (the encoder freezes while rotary is
// paused, so a live revolutionsInPass threshold can't be used to guard this).
// Reset whenever a bobbin's local pass tracking restarts (home(), start of
// bobbin B), since local pass numbers start over at 0 for each bobbin.
int lastHandledPassBoundary = -1;
// Steps for one full traversal pass across the layer width
const long LAYER_STEPS = lround((double)ROTATIONS_PER_LAYER * WIRE_DIAMETER / LINEAR_RES);

// Two-bobbin job sequence: wind bobbin A, wind a few slack turns in the gap,
// pause for the operator to hand-hook the wire onto bobbin B, then wind
// bobbin B (mirrored). Advanced automatically by pumpRotary() except the
// AWAIT_HOOK -> BOBBIN_B step, which requires a Start button press.
enum JobPhase { PHASE_BOBBIN_A, PHASE_SLACK, PHASE_AWAIT_HOOK, PHASE_BOBBIN_B, PHASE_DONE };
JobPhase jobPhase = PHASE_BOBBIN_A;

// Rotary drive state: IDLE (not moving) <-> RUNNING (spinning, button-started).
// Stopping is instant (no decel ramp), so there's no transitional state.
enum WindingState { WIND_IDLE, WIND_RUNNING };
WindingState windingState = WIND_IDLE;

unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_INTERVAL_MS = 100;  // update ~10x/sec, plenty for a human-readable display
/*=============================*/

/*========== OBJECTS ==========*/
// stepper linearStepper(LINEAR_STEP_PIN, LINEAR_DIR_PIN);  //OLD DEFINITION
AccelStepper LinearStepper(AccelStepper::DRIVER, LINEAR_STEP_PIN, LINEAR_DIR_PIN);
Encoder myenc(ENC_CHANNEL_A, ENC_CHANNEL_B);
// LiquidCrystal(rs, en, d4, d5, d6, d7)
LiquidCrystal lcd(12, 11, 4, 5, 6, 7);
AceButton startStopBtn(START_STOP_BTN_PIN);
AceButton homeBtn(HOME_BTN_PIN);
/*=============================*/

/*========== ROTARY TIMER (Timer1 hardware STEP pulse generator) ==========*/
// ROTARY_STEP_PIN (D10) is OC1B. Timer1 runs in CTC mode (TOP=ICR1, prescaler
// 8) with OC1B set to auto-toggle on every compare match, so STEP pulses are
// generated entirely in hardware, independent of loop() timing. This removes
// the ~1.7kHz ceiling a loop()-polled AccelStepper hit at high speed — the
// achievable step rate is no longer limited by how often loop() runs.
#define ROTARY_TIMER_PRESCALER   8
#define ROTARY_TIMER_HZ          (F_CPU / ROTARY_TIMER_PRESCALER)   // 2,000,000 Hz @ 16MHz/8

double rotaryCommandedFreq = 0.0;
unsigned long rotaryRampStartMicros = 0;
bool rotaryRampActive = false;

// Sets the STEP pulse frequency by rewriting Timer1's TOP (ICR1).
void rotarySetFrequency(double freqHz) {
    long icr = lround((double)ROTARY_TIMER_HZ / (2.0 * freqHz)) - 1;
    if (icr < 1) icr = 1;
    if (icr > 65535) icr = 65535;
    noInterrupts();
    ICR1 = (uint16_t)icr;
    interrupts();
    rotaryCommandedFreq = freqHz;
}

// Sets DIR, enables Timer1 hardware toggling of OC1B (D10), and begins the
// soft-start ramp from ROTARY_START_SPEED toward ROTARY_MAX_SPEED.
void rotaryTimerStart() {
    digitalWrite(ROTARY_DIR_PIN, ROTARY_RUN_DIRECTION > 0 ? HIGH : LOW);
    TCCR1A = (1 << COM1B0);                                // toggle OC1B on compare match
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);    // CTC, TOP=ICR1, prescaler=8
    OCR1B = 0;
    TCNT1 = 0;
    rotaryRampStartMicros = micros();
    rotaryRampActive = true;
    rotarySetFrequency(ROTARY_START_SPEED);
}

// Disconnects OC1B immediately — STEP pulses stop with no residual edges.
void rotaryTimerStop() {
    TCCR1A &= ~((1 << COM1B1) | (1 << COM1B0));   // OC1B disconnected, normal port operation
    TCCR1B = 0;                                    // stop the timer clock
    digitalWrite(ROTARY_STEP_PIN, LOW);
    rotaryRampActive = false;
    rotaryCommandedFreq = 0.0;
}

// Advances the soft-start ramp toward ROTARY_MAX_SPEED. Cheap to call every
// loop() iteration; stops touching the timer register once at cruise speed.
void rotaryRampUpdate() {
    if (!rotaryRampActive) return;
    double elapsedSec = (micros() - rotaryRampStartMicros) / 1000000.0;
    double freq = ROTARY_START_SPEED + ROTARY_ACCELERATION * elapsedSec;
    if (freq >= ROTARY_MAX_SPEED) {
        freq = ROTARY_MAX_SPEED;
        rotaryRampActive = false;
    }
    rotarySetFrequency(freq);
}

// Brief pass-boundary hold, distinct from a full stopWinding(): disconnects
// OC1B so STEP pulses stop, but leaves EN enabled so the driver keeps
// holding torque (the bobbin doesn't drift under wire tension) and leaves
// the ramp state untouched, so resuming continues at the same commanded
// speed instead of re-ramping from a standstill.
bool rotaryPaused = false;

void rotaryPauseSteps() {
    if (rotaryPaused) return;
    TCCR1A &= ~((1 << COM1B1) | (1 << COM1B0));   // OC1B disconnected, normal port operation
    digitalWrite(ROTARY_STEP_PIN, LOW);
    rotaryPaused = true;
}

void rotaryResumeSteps() {
    if (!rotaryPaused) return;
    TCNT1 = 0;
    TCCR1A |= (1 << COM1B0);   // reconnect OC1B toggle-on-compare
    rotaryPaused = false;
}
/*=============================*/

// Total revolutions turned since the encoder was last zeroed (home()).
double revolutionsWound() {
    return (double)myenc.read() / (RESOLUTION * 4.0);
}

// Redraws the normal running display (turn count / encoder count labels).
// Used after home() and whenever a phase transition (e.g. resuming for
// bobbin B) needs to replace a status message shown on the LCD.
void drawNormalDisplay() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("N_Turns:             ");
    lcd.setCursor(0, 1);
    lcd.print("Count:               ");
}

// Halts the rotary drive immediately (no decel ramp) and disables the driver
// so the shaft free-spins right away.
void stopWinding() {
    rotaryTimerStop();
    digitalWrite(ROTARY_EN_PIN, HIGH);
    windingState = WIND_IDLE;
}

// Services the rotary drive's soft-start ramp and advances the two-bobbin
// job through its phases as each phase's turn target is reached. Must be
// called every loop() iteration, and inside any blocking while-loop
// elsewhere (home()) — the STEP pulses themselves run in hardware regardless,
// but this still needs to run regularly to advance the ramp and catch phase
// boundaries in a timely fashion.
void pumpRotary() {
    if (windingState != WIND_RUNNING) return;

    double turns = revolutionsWound();

    if (jobPhase == PHASE_BOBBIN_A && turns >= TURNS_PER_BOBBIN) {
        jobPhase = PHASE_SLACK;
        LinearStepper.setAcceleration(LINEAR_ACCELERATION);
        LinearStepper.moveTo(lround(GAP_POS_MM / LINEAR_RES));
        printf("Phase: BOBBIN_A -> SLACK\n");
    } else if (jobPhase == PHASE_SLACK && (turns - TURNS_PER_BOBBIN) >= SLACK_TURNS) {
        jobPhase = PHASE_AWAIT_HOOK;
        stopWinding();
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Hook Bobbin B");
        lcd.setCursor(0, 1);
        lcd.print("Press Start");
        printf("Phase: SLACK -> AWAIT_HOOK (hook wire onto bobbin B, then press Start)\n");
        return;
    } else if (jobPhase == PHASE_BOBBIN_B
               && (turns - TURNS_PER_BOBBIN - SLACK_TURNS) >= TURNS_PER_BOBBIN) {
        jobPhase = PHASE_DONE;
        stopWinding();
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Winding Done");
        printf("Phase: BOBBIN_B -> DONE\n");
        return;
    }

    rotaryRampUpdate();
}

// Toggles the rotary winding drive on/off. Called from the left button.
// Starting ramps up smoothly; stopping is instant.
void toggleWinding() {
    if (windingState == WIND_IDLE) {
        if (jobPhase == PHASE_AWAIT_HOOK) {
            // Beginning bobbin B: reset per-bobbin pass tracking so its own
            // boundaries trigger fresh, and restore the normal display.
            jobPhase = PHASE_BOBBIN_B;
            atEndFlag = false;
            lastHandledPassBoundary = -1;
            drawNormalDisplay();
        } else if (jobPhase == PHASE_DONE) {
            printf("toggleWinding: job already complete, press Home to start a new one\n");
            return;
        }
        digitalWrite(ROTARY_EN_PIN, LOW);    // enable driver before commanding motion
        rotaryTimerStart();
        windingState = WIND_RUNNING;
        printf("toggleWinding: IDLE -> RUNNING (phase %d)\n", jobPhase);
    } else {
        stopWinding();
        printf("toggleWinding: RUNNING -> IDLE (instant stop)\n");
    }
}

// Drives linearStepper toward the endstop, backs off until the switch
// releases, moves to START_POS_MM, then zeros the encoder and all pass/turn
// tracking so encoder=0 corresponds to the winding start position — used
// both at startup and as the operator-triggered "reset turn count" action.
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

    // Move to winding start position, then zero all tracking so this
    // becomes the new zero-turn reference point.
    long startPositionSteps = lround(START_POS_MM / LINEAR_RES);
    LinearStepper.runToNewPosition(startPositionSteps);

    targetSteps = 0;
    atEndFlag = false;
    lastHandledPassBoundary = -1;
    jobPhase = PHASE_BOBBIN_A;
    myenc.write(0);
    prevPos = 0;
    LinearStepper.setCurrentPosition(0);    //Sets current position to position 0. Final Reference point.

    drawNormalDisplay();
}

void handleBtnEvent(AceButton* button, uint8_t eventType, uint8_t /*buttonState*/) {
    if (eventType != AceButton::kEventPressed) return;
    if (button->getPin() == START_STOP_BTN_PIN) {
        toggleWinding();
    } else if (button->getPin() == HOME_BTN_PIN) {
        // Reset turn count + re-home. Stop any in-progress winding first so
        // the carriage doesn't re-home while the bobbin is still spinning.
        if (windingState == WIND_RUNNING) {
            stopWinding();
        }
        home();
    }
}

// Drives LinearStepper through one bobbin's back-and-forth winding pattern,
// including the pass-boundary retract/pause. Shared by both bobbins:
//   revolutionsSinceStart — turns elapsed since THIS bobbin's winding began
//                           (bobbin A: revolutionsTotal; bobbin B: offset by
//                           TURNS_PER_BOBBIN + SLACK_TURNS).
//   positionOffsetSteps   — this bobbin's own zero reference along the
//                           leadscrew (0 for bobbin A, BOBBIN_B_START_POS_MM
//                           in steps for bobbin B).
//   mirrored              — flips which local pass is "forward", for a
//                           bobbin mounted the opposite way around.
void driveBobbinPass(double revolutionsSinceStart, long positionOffsetSteps, bool mirrored) {
    int localPass = (int)(revolutionsSinceStart / ROTATIONS_PER_LAYER);
    double revolutionsInPass = revolutionsSinceStart - (localPass * ROTATIONS_PER_LAYER);
    long stepsInPass = lround(revolutionsInPass * WIRE_DIAMETER / LINEAR_RES);
    bool forwardPass = mirrored ? (localPass % 2 != 0) : (localPass % 2 == 0);

    // At each pass boundary, pause the rotary drive itself (not just linear
    // motion) so the bobbin holds still while the carriage retracts. This
    // sidesteps racing the retract against a shrinking time window: winding
    // simply doesn't advance until the carriage is back in position, so it's
    // correct at any winding speed. Triggered once per distinct local pass
    // number (not a revolutionsInPass threshold window) — pausing rotary
    // freezes the encoder for the whole retract, so a live threshold would
    // either get stepped over between loop() iterations, or re-fire
    // repeatedly right after resuming since the encoder hasn't yet ticked
    // past it.
    if (!atEndFlag && localPass > 0 && localPass != lastHandledPassBoundary) {
        atEndFlag = true;
        lastHandledPassBoundary = localPass;
        // Retract from the bobbin end while winding is held. Direction
        // matches the pass just completed (same sign as forward travel).
        int retractDir = forwardPass ? 1 : -1;
        LinearStepper.setAcceleration((float)LINEAR_ACCELERATION * 4);
        LinearStepper.move(retractDir * lround(END_JOG_MM / LINEAR_RES));
        rotaryPauseSteps();
    }

    // Resume once the retract has actually finished.
    if (atEndFlag && LinearStepper.distanceToGo() == 0) {
        atEndFlag = false;
        LinearStepper.setAcceleration(LINEAR_ACCELERATION);
        rotaryResumeSteps();

        long formulaTarget = forwardPass ? stepsInPass : (LAYER_STEPS - stepsInPass);

        // Relabel the carriage's current (retracted) physical position as
        // that target. No motion occurs — just a coordinate reset.
        LinearStepper.setCurrentPosition(positionOffsetSteps + formulaTarget);
    }

    if (!atEndFlag) {
        long localTarget = forwardPass ? stepsInPass : (LAYER_STEPS - stepsInPass);
        targetSteps = localTarget;
        LinearStepper.moveTo(positionOffsetSteps + localTarget);
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
    //delay(2000);

    Serial.begin(115200);
    Serial.begin(115200);

    LinearStepper.setMaxSpeed(LINEAR_MAX_SPEED);
    LinearStepper.setAcceleration(LINEAR_ACCELERATION);

    pinMode(ENDSTOP_PIN, INPUT_PULLUP);
    pinMode(START_STOP_BTN_PIN, INPUT_PULLUP);
    pinMode(HOME_BTN_PIN, INPUT_PULLUP);
    pinMode(ROTARY_STEP_PIN, OUTPUT);    // AccelStepper no longer owns this pin; drive it ourselves
    pinMode(ROTARY_DIR_PIN, OUTPUT);
    pinMode(ROTARY_EN_PIN, OUTPUT);
    digitalWrite(ROTARY_EN_PIN, HIGH);   // start disabled: free-spinning, no holding torque
    ButtonConfig::getSystemButtonConfig()->setEventHandler(handleBtnEvent);

    home(); //Homes the linear drive carriage; also draws the N_Turns/Count LCD labels
}

void loop() {

    startStopBtn.check();
    homeBtn.check();

    pumpRotary();
    LinearStepper.run();

    // TEMP DEBUG: loopHz confirms loop() is no longer the bottleneck (STEP
    // pulses now come from Timer1 hardware, independent of loop() timing).
    // commandedHz is what Timer1 is actually being told to output.
    // encRevPerSec is measured straight from the encoder — real physical
    // shaft speed. If commandedHz climbs toward ROTARY_MAX_SPEED but
    // encRevPerSec plateaus/stays low, the motor is stalling mechanically
    // (current/torque/microstep issue), not a firmware limitation.
    // Remove once the speed issue is diagnosed.
    static unsigned long rateLoopCount = 0;
    static unsigned long lastRateCheck = 0;
    static long lastEncForRate = 0;
    rateLoopCount++;
    if (millis() - lastRateCheck >= 1000) {
        long encNow = myenc.read();
        printf("loopHz=%lu commandedHz=%.0f encRevPerSec=%.2f windingState=%d\n",
               rateLoopCount, rotaryCommandedFreq, (encNow - lastEncForRate) / (RESOLUTION * 4.0),
               windingState);
        rateLoopCount = 0;
        lastEncForRate = encNow;
        lastRateCheck = millis();
    }

    long newPos = myenc.read();
    double revolutionsTotal = (double)newPos / (RESOLUTION * 4.0); //Absolute Total Revolutions Completed
    int passNumber = (int)(revolutionsTotal / ROTATIONS_PER_LAYER);  //Rounds Down Always. Begins at 0 (LtR); telemetry only, driveBobbinPass() computes its own local pass number

    if (newPos != prevPos) {
        prevPos = newPos;

        if (millis() - lastDisplayUpdate >= DISPLAY_INTERVAL_MS) {
            lastDisplayUpdate = millis();

            char buf[9];
            lcd.setCursor(8, 0);
            dtostrf(revolutionsTotal, 7, 2, buf);
            lcd.print(buf);

            lcd.setCursor(7, 1);
            dtostrf((double)newPos, 8, 0, buf);
            lcd.print(buf);

            printf("Count: %ld\t Pass: %d\t Turns: %.2f\n", newPos, passNumber, revolutionsTotal);
        }
    }

    // Drive whichever bobbin is currently active. PHASE_SLACK/AWAIT_HOOK/DONE
    // need no per-loop linear tracking — the carriage just holds wherever it
    // last was sent (the gap position, or its final position).
    if (jobPhase == PHASE_BOBBIN_A) {
        driveBobbinPass(revolutionsTotal, 0, false);
    } else if (jobPhase == PHASE_BOBBIN_B) {
        double revolutionsSinceBobbinB = revolutionsTotal - TURNS_PER_BOBBIN - SLACK_TURNS;
        driveBobbinPass(revolutionsSinceBobbinB, lround(BOBBIN_B_START_POS_MM / LINEAR_RES), BOBBIN_B_MIRRORED);
    }
}

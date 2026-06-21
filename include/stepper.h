#ifndef stepper_h
#define stepper_h

/*
/   Stepper Controller Model: TMC2209
/   Pinout:
/       P0 = Enable
/       P1 = MS1 (MircoStep Resolution)
/       P2 = MS2 (MircoStep Resolution)
/       P3 = PDN
/       P4 = PDN
/       P5 = CLK (Tie to GND for internal clock)
/       P6 = STEP (Single Step Pulse)
/       P7 = DIR (Step Direction)
/       P8 = VM (Stepper Motor Reference Voltage)
/       P9 = GND
/       P10 = A2
/       P11 = A1
/       P12 = B1
/       P13 = B2
/       P14 = A2
/       P15 = VDD   (5V)
/       P16 = GND
/   Resolution:
/       00 = 1/8
/       01 = 1/32
/       10 = 1/64
/       11 = 1/16
/   0.01mm linear travel per step
*/

#include <Arduino.h>

//TODO Adjust pin numbers as required

class stepper{
    public:

    stepper(int Step_Pin, int Direction_Pin){
        _stepPin = Step_Pin;
        _dirPin = Direction_Pin;
    }

    void enable(){
        pinMode(_stepPin, OUTPUT);
        pinMode(_dirPin, INPUT);
    }

    void step(int direction, int num_step){
        
        if(direction == 1){
            //Turn CCW?
            //TODO Verify direction of turn for HIGH & LOW on DIR Pin
            for(int i = 0; i < num_step; i++){
                digitalWrite(_stepPin, HIGH);
                delayMicroseconds(160);
                digitalWrite(_stepPin, LOW);
                delayMicroseconds(160);
            }
        }
        else{
            // Turn CW?
            //TODO Verify direction of turn for HIGH & LOW on DIR Pin
            for(int i = 0; i < num_step; i++){
                digitalWrite(_stepPin, HIGH);
                delayMicroseconds(160);
                digitalWrite(_stepPin, LOW);
                delayMicroseconds(160);
            }
        }
    }

    private:
    int _stepPin;
    int _dirPin;
};

#endif
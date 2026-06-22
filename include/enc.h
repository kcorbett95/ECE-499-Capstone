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

#include <Arduino.h>
#include <LibPrintf.h>

#define ENC_CHANNEL_A 2
#define ENC_CHANNEL_B 3
#define ENC_CHANNEL_INDX 4
#define RESOLUTION 2048
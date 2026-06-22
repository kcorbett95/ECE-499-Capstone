#include <Arduino.h>
#include "enc.h"
#include <LibPrintf.h>
#include <Encoder.h>
#include <TMCStepper.h>
#include <LiquidCrystal.h>

//LCD Pin	  Connection	  Uno	        Nano
///VSS	    GND	          GND header	GND pin
///VDD	    5V	          5V header	  5V pin
///RS       D12	          D12	        D12
///EN	      D11	          D11	        D11
///D4	      D5	          D5	        D5
///D5	      D4	          D4	        D4
///D6	      D3	          D3	        D3
///D7	      D2	          D2	        D2

// LiquidCrystal(rs, en, d4, d5, d6, d7)
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);


int prevPos = 0;

Encoder myenc(ENC_CHANNEL_A, ENC_CHANNEL_B);

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
    lcd.print("N_Turns: ");

    lcd.setCursor(0, 1);
    lcd.print("Count: ");

    Serial.begin(9600);

    lcd.setCursor(0, 1);
    lcd.print("Capstone 2026");z
}

void loop() {

    double newPos, revolutions;

    newPos = myenc.read();

    if(newPos != prevPos){
        revolutions = newPos/(2048.0*4.0);
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
  
}
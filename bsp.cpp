#include "Arduino.h"
#include "bsp.h"

volatile int counter = 0;
volatile int aState;
volatile int aLastState;

void bsp_ctor(void) {
  Serial.begin(9600);
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_3, OUTPUT);
  pinMode(outputA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(outputA), encoderInterrupt, CHANGE);
  pinMode(outputB, INPUT);
}

/* this interrupt will count rotate rate of encoder */
void encoderInterrupt() {
  aState = digitalRead(outputA); // Reads the "current" state of the outputA
  // If the previous and the current state of the outputA are different, that means a Pulse has occured
  if (aState != aLastState) {
    // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
    if (digitalRead(outputB) != aState) {
      counter ++;
    } else {
      counter --;
    }
    //Serial.print("Position: ");
    //Serial.println(counter);
  }
  aLastState = aState; // Updates the previous state of the outputA with the current state
}

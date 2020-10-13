#include "Arduino.h"
#include "bsp.h"
#include "lineSensor.hpp"

volatile int counter = 0;
volatile int aState;
volatile int aLastState;

void bsp_ctor(void) {
  Serial.begin(9600);
  pinMode(L_DIR_PIN, OUTPUT);
  pinMode(R_DIR_PIN, OUTPUT);
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


void smartMotorControl(int r_motor_speed, int l_motor_speed)
{
  if (abs(r_motor_speed) > 255 || abs(l_motor_speed) > 255) {
    Serial.print("non-valid values have entered!");
    return;
  }
  if ( (r_motor_speed > 0) && (l_motor_speed > 0)) {
    digitalWrite(R_DIR_PIN, HIGH);
    analogWrite(R_PWM_PIN, r_motor_speed);
    digitalWrite(L_DIR_PIN, HIGH);
    analogWrite(L_PWM_PIN, l_motor_speed);
  } else if ( (r_motor_speed > 0) && (l_motor_speed < 0)) {
    digitalWrite(R_DIR_PIN, HIGH);
    analogWrite(R_PWM_PIN, r_motor_speed);
    digitalWrite(L_DIR_PIN, LOW);
    analogWrite(L_PWM_PIN, -l_motor_speed);
  } else if ( (r_motor_speed < 0) && (l_motor_speed > 0)) {
    digitalWrite(R_DIR_PIN, LOW);
    analogWrite(R_PWM_PIN, -r_motor_speed);
    digitalWrite(L_DIR_PIN, HIGH);
    analogWrite(L_PWM_PIN, l_motor_speed);
  } else if ( (r_motor_speed < 0) && (l_motor_speed < 0)) {
    digitalWrite(R_DIR_PIN, LOW);
    analogWrite(R_PWM_PIN, -r_motor_speed);
    digitalWrite(L_DIR_PIN, LOW);
    analogWrite(L_PWM_PIN, -l_motor_speed);
  } else if ( (r_motor_speed == 0) && (l_motor_speed == 0)) {
    digitalWrite(R_DIR_PIN, HIGH);
    analogWrite(R_PWM_PIN, r_motor_speed);
    digitalWrite(L_DIR_PIN, HIGH);
    analogWrite(L_PWM_PIN, l_motor_speed);
  }
}

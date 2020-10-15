#include "Arduino.h"
#include "..\inc\bsp.h"
#include "..\inc\lineSensor.hpp"

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

boolean nonBlockingDelay(unsigned long dly) {

    static unsigned long delay_last_timestamp = millis();
    static boolean isEnteredFirst = true;

    if (isEnteredFirst) {
        delay_last_timestamp = millis();
        isEnteredFirst = false;
    }
    
    unsigned long delay_time_now = millis();
    if ((delay_time_now - delay_last_timestamp) > dly)
    {
        isEnteredFirst = true;
        return true;
    }
    else {
        return false;
    }
    
}

float readMotorSpeedTask(void) {
    /* In this task we are getting our current
       motor speed.
    */
    static unsigned long mSpeed_last_timestamp = millis();
    static int last_counted_val = 0;
    unsigned long mSpeed_time_now = millis();

    float counterDiff = counter - last_counted_val;
    float velocity = counterDiff / (float)(mSpeed_time_now - mSpeed_last_timestamp);
    velocity = MAX_MOTOR_POWER * velocity;
    last_counted_val = counter;
    mSpeed_last_timestamp = millis();
    return velocity;
}

void smartMotorControl(int r_motor_speed, int l_motor_speed)
{
  if (abs(r_motor_speed) > 255 || abs(l_motor_speed) > 255) {
    Serial.print("non-valid values have been entered!");
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

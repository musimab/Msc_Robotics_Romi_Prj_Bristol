#include "bsp.h"
#include "Romi.h"

volatile float velocity = 0.0;

uint8_t currentState = IDLE_STATE;

void lineSensingTask(void) {
  static bool local_led1_check = true;
  if (local_led1_check)
  {
    digitalWrite(LED_1, HIGH);
    local_led1_check = false;
  } else {
    digitalWrite(LED_1, LOW);
    local_led1_check = true;
  }
}

void readMotorSpeedTask(void) {
  /* In this task we are getting our current
     motor speed */
  static int lastCountedVal = 0;
  float counterDiff = counter - lastCountedVal;
  velocity = counterDiff / (float)readMotorSpeedIns.getElapsedTime();

  Serial.print("speed:");
  Serial.println(10 * velocity);

  lastCountedVal = counter;
}

void setup() {
  bsp_ctor();
  aLastState = digitalRead(outputA);
  GO_HANDLE(IDLE_STATE); // start with handling IDLE state
}

void loop() {
  taskInsert::executeTasks();
  switch (currentState) {

    case IDLE_STATE: {

        GO_HANDLE(READ_LINE_SENSOR);
        break;
      }

    case READ_LINE_SENSOR: {
        if (lineSensingIns.callMyTask()) {
          GO_HANDLE(READ_MOTOR_SPEED);
        }
        break;
      }

    case READ_MOTOR_SPEED: {

        if (readMotorSpeedIns.callMyTask()) {
          GO_HANDLE(IDLE_STATE);
        }
        break;
      }

    case ADJUST_MOTOR_SPEED: {

        GO_HANDLE(IDLE_STATE);
        break;
      }

    default : {
        GO_HANDLE(IDLE_STATE);
        break;
      }
  }
}



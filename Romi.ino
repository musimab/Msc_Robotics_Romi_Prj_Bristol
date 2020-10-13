/* ROMI PROJECT  
   University Of Bristol
   Robotics System
   Furkan Cam
*/

#include "bsp.h"
#include "Romi.h"
#include "lineSensor.hpp"

/* we have created with default pins */
lineSensor<uint8_t> lineSensorIns;
sensorMotorPowers sMPower;

volatile float velocity = 0.0;

uint8_t currentState = IDLE_STATE;

/* Implemented Tasks */
void lineSensingTask(void);
void readMotorSpeedTask(void);

/* Define all used instances for
   non-blocking millis functions */
taskInsert lineSensingTaskIns(lineSensingTask, 20);
taskInsert readMotorSpeedTaskIns(readMotorSpeedTask, 50);

void lineSensingTask(void) {
  /* if the obtained values are higher than
     the determined threshold, we are on line
     and we should implement our motor speed 
     according to this situation. 
     */
  if (lineSensorIns.onLine(LINE_TRESHHOLD))
    GO_HANDLE(ON_LINE_STATE);
  GO_HANDLE(OFF_LINE_STATE);
}

void readMotorSpeedTask(void) {
  /* In this task we are getting our current
     motor speed.
     */
  static int lastCountedVal = 0;
  float counterDiff = counter - lastCountedVal;
  velocity = counterDiff / (float)readMotorSpeedTaskIns.getElapsedTime();

  lastCountedVal = counter;
}

void onLineState(void) {
  lineSensorIns.calculateMotorSpeed(sMPower);

  smartMotorControl((int)sMPower.left_motor_power, (int)sMPower.right_motor_power);

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
        /* is there any line detected ?
           call lineSensingTask func in 20ms freq
        */
        lineSensingTaskIns.callMyTask(); 
        break;
      }

    case READ_MOTOR_SPEED: {

        if (readMotorSpeedTaskIns.callMyTask()) {
          GO_HANDLE(IDLE_STATE);
        }
        break;
      }

    case ON_LINE_STATE: {

        onLineState();
    
        GO_HANDLE(IDLE_STATE);
        break;
      }

    case OFF_LINE_STATE: {

        GO_HANDLE(IDLE_STATE);
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



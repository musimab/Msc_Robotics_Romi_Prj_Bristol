#ifndef _ROMI_H_
#define _ROMI_H_

#include "nonBlockingMillis.h"

#define GO_HANDLE(x) { currentState = x; }

/* Implemented Tasks */
void lineSensingTask(void);
void readMotorSpeedTask(void);

/* Define all used instances */
taskInsert lineSensingIns(lineSensingTask, 50);
taskInsert readMotorSpeedIns(readMotorSpeedTask, 50);

enum event_states {
  IDLE_STATE,
  READ_LINE_SENSOR,
  READ_MOTOR_SPEED,
  ADJUST_MOTOR_SPEED,
};

extern uint8_t currentState;

#endif _ROMI_H_

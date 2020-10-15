#ifndef _ROMI_H_
#define _ROMI_H_

#include "nonBlockingMillis.h"

#define GO_HANDLE(x) { currentState = x; }

enum ROMI_TASKS {
  IDLE_STATE,
  FIND_LINE,
  READ_LINE_SENSOR,
  OFF_LINE_STATE,
  ON_LINE_STATE,
  READ_MOTOR_SPEED,
  ADJUST_MOTOR_SPEED,
};

extern uint8_t currentState;

#endif _ROMI_H_

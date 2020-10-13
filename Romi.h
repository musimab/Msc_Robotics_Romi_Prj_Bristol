#ifndef _ROMI_H_
#define _ROMI_H_

#include "nonBlockingMillis.h"

#define GO_HANDLE(x) { currentState = x; }
#define LINE_TRESHHOLD 500

enum event_states {
  IDLE_STATE,
  READ_LINE_SENSOR,
  OFF_LINE_STATE,
  ON_LINE_STATE,
  READ_MOTOR_SPEED,
  ADJUST_MOTOR_SPEED,
};

extern uint8_t currentState;

#endif _ROMI_H_

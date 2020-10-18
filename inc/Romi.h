#ifndef _ROMI_H_
#define _ROMI_H_

#include "nonBlockingMillis.h"

#define GO_HANDLE(x) { current_state = x; }
#define NON_BLOCKING_DELAY(n, t)  \
		current_state = NON_BLOCKING_DELAY_STATE; \
	    next_state = n;  \
		blocking_time = t  \

enum ROMI_TASKS {
  IDLE_STATE,
  NON_BLOCKING_DELAY_STATE,
  FIND_LINE,
  READ_LINE_SENSOR,
  OFF_LINE_STATE,
  ON_LINE_STATE,
  READ_MOTOR_SPEED,
  ADJUST_MOTOR_SPEED,
};

extern uint8_t current_state;
extern uint8_t next_state;
extern uint32_t blocking_time;

#endif _ROMI_H_

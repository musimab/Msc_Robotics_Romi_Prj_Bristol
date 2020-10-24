#ifndef _ROMI_H_
#define _ROMI_H_

#include "nonBlockingMillis.h"

#define GO_HANDLE(x) { current_state = x; }
#define WAIT_THEN_GO_STATE(n, t)  \
		current_state = NON_BLOCKING_DELAY_STATE; \
	    next_state = n;  \
		blocking_time = t  \

#define WAIT_NONBLOCKING_MS(ms) if (!nonBlockingDelay(ms)) { \
         			 GO_HANDLE(IDLE_STATE);  \
         			 break; \
       				 } 
       				 
#define WAIT_NONBLOCKING_SANE_MS(ms, state) if (!nonBlockingDelay(ms)) { \
					GO_HANDLE(state); \
         			 break; \
       				 }        				 

enum ROMI_TASKS {
  IDLE_STATE,
  NON_BLOCKING_DELAY_STATE,
  FIND_LINE,
  JOIN_LINE,
  READ_LINE_SENSOR,
  OFF_LINE_STATE,
  ON_LINE_STATE,
  STOP_MOTOR_STATE,
  READ_MOTOR_SPEED,
  ADJUST_MOTOR_SPEED,
};

extern uint8_t current_state;
extern uint8_t next_state;
extern uint32_t blocking_time;

#endif _ROMI_H_

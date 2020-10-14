#ifndef _NON_BLOCKING_MILLIS_
#define _NON_BLOCKING_MILLIS_

#include "stdint.h"

/* This class is designed for handling multi
   tasks based system. It is just provide us
   non-blocking mechanism, but in more convinient
   way */

class taskInsert {
  private:
    typedef void(*taskHandler)(void);
    unsigned long freqOfTask{ 0 };
    unsigned long last_timestamp{ 0 };
    unsigned long elapsed_time{ 0 };
    taskHandler myTaskFunc{ nullptr };
    static unsigned long time_now;
  public:
    /*  Constructor of this class is taking the function which is
        intended to work for a certain frequency and the frequency
        of this function.
    */
    taskInsert(taskHandler fptr, uint16_t freq) : freqOfTask{ freq },
      myTaskFunc{ fptr } {
      // Get the first millis value before start this task
      last_timestamp = millis();
    }

    // This function will be calling the desired function for a certain freq
    bool callMyTask(void);

    // This function is to get the current system working time
    static void executeTasks(void) {
      // Get how much time has passed right now.
      time_now = millis();
    }

    unsigned long getElapsedTime(void);
};

#endif _NON_BLOCKING_MILLIS_



#include "Arduino.h"
#include "..\inc\nonBlockingMillis.h"

unsigned long taskInsert::time_now = 0;

/* To call all function in a spesific time,
   this function should be used */

bool taskInsert::callMyTask(void) {
  elapsed_time = time_now - last_timestamp;
  if (elapsed_time > freqOfTask)
  {
    last_timestamp = millis();
    myTaskFunc();
    // inform the user that the task has been implemented
    return true; 
  }
  // inform the user that the task has not been implemented
  return false;
}

// get the elapsed time value
unsigned long taskInsert::getElapsedTime(void) {
  return elapsed_time;
}


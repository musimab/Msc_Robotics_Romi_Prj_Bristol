#include "nonBlockingMillis.h"

unsigned long taskInsert::time_now = 0;

/* To call all function in a spesific time,
   this function should be used */
   
void taskInsert::callMyTask(void) {
  elapsed_time = time_now - last_timestamp;
  if (elapsed_time > freqOfTask)
  {
    last_timestamp = millis();
    myTaskFunc();
  }
}

// get the elapsed time value 
unsigned long taskInsert::getElapsedTime(void) {
  return elapsed_time;
}


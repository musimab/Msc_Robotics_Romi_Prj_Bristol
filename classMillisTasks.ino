#include "nonBlockingMillis.h"

#define LED_1 6
#define LED_2 7
#define LED_3 8
#define outputA 2 // interrupt pin
#define outputB 10

void task_1(void);
void task_2(void);
void task_3(void);
void task_4(void);

taskInsert myTask1(task_1, 100);
taskInsert myTask2(task_2, 200);
taskInsert myTask3(task_3, 300);
taskInsert myTask4(task_4, 50);

volatile int counter = 0;
volatile int aState;
volatile int aLastState;
volatile float velocity = 0.0;

void task_1(void) {
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

void task_2(void) {
  static bool local_led2_check = true;
  if (local_led2_check)
  {
    digitalWrite(LED_2, HIGH);
    local_led2_check = false;
  } else {
    digitalWrite(LED_2, LOW);
    local_led2_check = true;
  }
}

void task_3(void) {
  static bool local_led3_check = true;
  if (local_led3_check)
  {
    digitalWrite(LED_3, HIGH);
    local_led3_check = false;
  } else {
    digitalWrite(LED_3, LOW);
    local_led3_check = true;
  }
}

void task_4(void) {
  /* In this task we are gettin our current
     motor speed */
  static int lastCountedVal = 0;
  float counterDiff = counter - lastCountedVal;
  velocity = counterDiff / (float)myTask4.getElapsedTime();

  Serial.print("speed:");
  Serial.println(10*velocity);

  lastCountedVal = counter;
}

void setup() {
  Serial.begin(9600);
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_3, OUTPUT);
  pinMode(outputA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(outputA), encoderInterrupt, CHANGE);
  pinMode(outputB, INPUT);
  aLastState = digitalRead(outputA);
}

void loop() {
  taskInsert::executeTasks();
  myTask1.callMyTask();
  myTask2.callMyTask();
  myTask3.callMyTask();
  myTask4.callMyTask();
}

void encoderInterrupt() {
  aState = digitalRead(outputA); // Reads the "current" state of the outputA
  // If the previous and the current state of the outputA are different, that means a Pulse has occured
  if (aState != aLastState) {
    // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
    if (digitalRead(outputB) != aState) {
      counter ++;
    } else {
      counter --;
    }
    //Serial.print("Position: ");
    //Serial.println(counter);
  }
  aLastState = aState; // Updates the previous state of the outputA with the current state
}


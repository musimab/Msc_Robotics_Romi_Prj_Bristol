/* ROMI PROJECT
  University Of Bristol
  Robotics System
  Furkan Cam
*/

#include "inc/bsp.h"
#include "inc/Romi.h"
#include "inc/lineSensor.hpp"
#include "inc/pidLib.h"
#include "inc/motor.hpp"
#include "inc/kinematics.hpp"

/* we have created with default pins */

#define KP_VAL 1   // increase response time, but it'll increase oscillation also
#define KI_VAL 1 // minimise the total error 
#define KD_VAL 1   // if there is a huge changing

#define M_SPEED 15

/* Motor instances */
myMotor<uint8_t> leftMotorInstance(L_DIR_PIN, L_PWM_PIN);
myMotor<uint8_t> rightMotorInstance(R_DIR_PIN, R_PWM_PIN);
/* Line sensor instance */
lineSensor<uint8_t> lineSensorIns;

sensorMotorPowers sMPower;
PID pidForLineFollowing(0, 0, 0);
PID pidForLeft(KP_VAL, KI_VAL, KD_VAL);
PID pidForRight(KP_VAL, KI_VAL, KD_VAL);
uint8_t current_state;
uint8_t next_state = IDLE_STATE;
uint32_t blocking_time {0};

int left_motor_speed {M_SPEED};
int right_motor_speed {M_SPEED};
/* Implemented Tasks */

/* Define all used instances for
   non-blocking millis functions */
void lineSensingTask(void);
taskInsert lineSensingTaskIns(lineSensingTask, 20);

void offLineStateTask(void);
taskInsert offLineStateTaskIns(offLineStateTask, 15);

// start Romi from origin point
kinematics knm(0, 0);

void lineSensingTask(void) {
  /* if the obtained values are higher than
     the determined threshold, we are on line
     and we should implement our motor speed
     according to this situation.
  */
  if (lineSensorIns.isOnLine()) {
    if (lineSensorIns.isLeftOnline() || lineSensorIns.isRightOnline()) {
      //if our robot on the right or left of the line, set it on the midst
      lineSensorIns.calculateMotorSpeed(sMPower);
      //we have to saperate the who wheels' encoder rate each other
      //smartMotorControl(pidForLineFollowing.updateValue((int)sMPower.left_motor_power, 1/*readMotorSpeedTask()*/),
      //                pidForLineFollowing.updateValue((int)sMPower.right_motor_power, 1/*readMotorSpeedTask()*/));

    } else {
      //Follow the line with smooth motor speed
      //float motor_speed = pidForLineFollowing.updateValue(50, 1/*readMotorSpeedTask()*/);
      //smartMotorControl(motor_speed, motor_speed);
    }
    GO_HANDLE(ON_LINE_STATE);
  } else {
    GO_HANDLE(OFF_LINE_STATE);
  }
}

void offLineStateTask() {

  /*if( knm.get_angle() > 0.2) {
    left_motor_speed -= 1;
    } else {
    left_motor_speed += 1;
    }*/

  leftMotorInstance.motorControl(pidForLeft.updateValue(left_motor_speed, leftMotorInstance.readMotorSpeed(&count_e0)));
  rightMotorInstance.motorControl(pidForRight.updateValue(right_motor_speed, rightMotorInstance.readMotorSpeed(&count_e1)));
}

void setup() {
  bsp_ctor();
  lineSensorIns.setTreshold(LINE_TRESHOLD);
  pidForLineFollowing.reset();
  pidForRight.reset();
  pidForLeft.reset();
  lineSensorIns.calibrate(1000);
  delay(2000);

  GO_HANDLE(IDLE_STATE); // start with handling IDLE state
}

void loop() {
  taskInsert::executeTasks();
  knm.kinematicupdate(count_e0, count_e1);
  switch (current_state) {
    case IDLE_STATE: {

        GO_HANDLE(READ_LINE_SENSOR);
        break;
      }

    case READ_LINE_SENSOR: {
        if (lineSensorIns.isOnLine()) {
          /* we have to know is the robot on line or just
            entiring a line from the right side, if so we
            should turn robot right first */
          // give a sound with buzzer
          Serial.println("online!!!");
          knm.resetDistanceFrom();
          GO_HANDLE(ON_LINE_STATE);
        } else {
          left_motor_speed = M_SPEED;
          right_motor_speed = M_SPEED;
          offLineStateTaskIns.callMyTask();
        }
        break;
      }

    case ON_LINE_STATE: {
        if (lineSensorIns.isOnLine())
        {
          GO_HANDLE(ON_LINE_STATE);
        } else {
          Serial.println("offline!!!");
          if (knm.getDistanceFrom() > 12.5) {
            leftMotorInstance.motorControl(0);
            rightMotorInstance.motorControl(0);
            GO_HANDLE(FIND_LINE);
          }
        }
        //lineSensingTaskIns.callMyTask();

        break;
      }

    case FIND_LINE: {
        WAIT_NONBLOCKING_SANE_MS(1400, FIND_LINE);
        GO_HANDLE(JOIN_LINE);

        break;
      }
      
    case JOIN_LINE: {
        knm.printVals();
        if (knm.get_angle() < 90) {
          left_motor_speed = M_SPEED;
          right_motor_speed = -M_SPEED;
          offLineStateTaskIns.callMyTask();
          GO_HANDLE(FIND_LINE);
        } else {
          // Follow the line
          leftMotorInstance.motorControl(0);
          rightMotorInstance.motorControl(0);
          GO_HANDLE(STOP_MOTOR_STATE);
        }
        GO_HANDLE(JOIN_LINE);
        break;
      }

    case OFF_LINE_STATE: {
        GO_HANDLE(ADJUST_MOTOR_SPEED);
        break;
      }

    case STOP_MOTOR_STATE: {
        leftMotorInstance.motorControl(0);
        rightMotorInstance.motorControl(0);
        GO_HANDLE(STOP_MOTOR_STATE);
        break;
      }

    case ADJUST_MOTOR_SPEED: {
        left_motor_speed = M_SPEED;
        right_motor_speed = M_SPEED;
        offLineStateTaskIns.callMyTask();
        GO_HANDLE(IDLE_STATE);
        break;
      }

    case NON_BLOCKING_DELAY_STATE: {
        if (!nonBlockingDelay(blocking_time)) {
          GO_HANDLE(IDLE_STATE);
        } else {
          GO_HANDLE(next_state);
        }
        break;
      }

    default : {
        GO_HANDLE(IDLE_STATE);
        break;
      }
  }
}



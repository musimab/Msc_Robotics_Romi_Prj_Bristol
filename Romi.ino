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
int turning_angle {0};
/* Implemented Tasks */

/* Define all used instances for
   non-blocking millis functions */
void lineSensingTask(void);
taskInsert lineSensingTaskIns(lineSensingTask, 25);

void motorHandleTask(void);
taskInsert motorHandleTaskIns(motorHandleTask, 35);

// start Romi from origin point
kinematics knm(0, 0);

void lineSensingTask(void) {
  /* if the obtained values are higher than
     the determined threshold, we are on line
     and we should implement our motor speed
     according to this situation.
  */
  if (lineSensorIns.isLeftOnline() || lineSensorIns.isRightOnline()) {
    //if our robot on the right or left of the line, set it on the midst
    lineSensorIns.calculateMotorSpeed(sMPower);

    leftMotorInstance.motorControl(pidForLeft.updateValue(M_SPEED - sMPower.left_motor_power, leftMotorInstance.readMotorSpeed(&count_e0)) / 5.0);
    rightMotorInstance.motorControl(pidForRight.updateValue(M_SPEED - sMPower.right_motor_power, rightMotorInstance.readMotorSpeed(&count_e1)) / 5.0);

  }
}

bool motorTurning(float degree) {
  static boolean isAngleEnteredFirst = true;
  static float target_angle = 0;
  static float angle_start_point = 0;
  float system_angle = knm.get_angle();

  if (isAngleEnteredFirst) {
    isAngleEnteredFirst = false;
    angle_start_point = system_angle;
    target_angle = angle_start_point + degree;
  }

  Serial.print("angle Start Point: "); Serial.print(angle_start_point);
  Serial.print(" - target angle: "); Serial.print(target_angle);
  Serial.print(" - system angle: "); Serial.println(system_angle);

  if (((degree > 0) && (system_angle > 0)) || ((degree > 0) && (system_angle < 0))) {
    if (system_angle < target_angle)
    {
      Serial.println("state1............................ ");
      left_motor_speed = M_SPEED;
      right_motor_speed = -M_SPEED;
    } else {
      isAngleEnteredFirst = true;
      return false;
    }
  } else if (((degree < 0) && (system_angle > 0)) || ((degree < 0) && (system_angle < 0))) {
    if (system_angle > target_angle)
    {
      Serial.println("state2............................ ");
      left_motor_speed = -M_SPEED;
      right_motor_speed = M_SPEED;
    } else {
      isAngleEnteredFirst = true;
      return false;
    }
  }

  Serial.print("left Motor speed: "); Serial.print(left_motor_speed);
  Serial.print(" - right motor speed: "); Serial.print(right_motor_speed);
  motorHandleTaskIns.callMyTask();
  return true;
}

void motorHandleTask() {

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
          Serial.println("read line else -state");
          left_motor_speed = M_SPEED;
          right_motor_speed = M_SPEED;
          motorHandleTaskIns.callMyTask();
        }
        break;
      }

    case ON_LINE_STATE: {
        Serial.println("online-state");
        if (lineSensorIns.isOnLine())
        {
          GO_HANDLE(ON_LINE_STATE);
        } else {
          //Off-line state
          //Let system go beyond the line for the settling
          if (knm.getDistanceFrom() > 12.5) {
            leftMotorInstance.motorControl(0);
            rightMotorInstance.motorControl(0);
            turning_angle = 90;
            GO_HANDLE(FIND_LINE);
          }
        }
        break;
      }

    case FIND_LINE: {
        Serial.println("find line-state");
        WAIT_NONBLOCKING_SANE_MS(500, FIND_LINE);
        GO_HANDLE(TURN_ROMI);
        break;
      }

    case TURN_ROMI: {
        knm.printVals();
        if ( motorTurning(turning_angle)) {
          GO_HANDLE(TURN_ROMI);
        } else {
          //Check first whether line is continuing
          leftMotorInstance.motorControl(0);
          rightMotorInstance.motorControl(0);
          GO_HANDLE(CHECK_LINE_STATE);
        }
        break;
      }

    case CHECK_LINE_STATE: {
        Serial.println("CHECK_LINE_STATE!!!");
        if (lineSensorIns.isOnLine()) {
          WAIT_NONBLOCKING_SANE_MS(500, CHECK_LINE_STATE);
          GO_HANDLE(SETTLING_LINE_STATE);
        } else {
          turning_angle = -90;
          GO_HANDLE(FIND_LINE);
        }
        break;
      }

    case SETTLING_LINE_STATE: {
        Serial.println("settling line-state");
        if (lineSensorIns.isOnLine()) {
          lineSensingTaskIns.callMyTask();
          GO_HANDLE(SETTLING_LINE_STATE);
        } else {
          leftMotorInstance.motorControl(0);
          rightMotorInstance.motorControl(0);
          knm.resetDistanceFrom();
          turning_angle = 90;
          GO_HANDLE(FOLLOW_LINE_STATE);
        }
        break;
      }

    case FOLLOW_LINE_STATE: {
        Serial.println("FOLLOW_LINE_STATE-state");
        WAIT_NONBLOCKING_SANE_MS(100, FOLLOW_LINE_STATE);
        leftMotorInstance.motorControl(-15);
        rightMotorInstance.motorControl(-15);
        GO_HANDLE(ON_LINE_STATE);
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
        motorHandleTaskIns.callMyTask();
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



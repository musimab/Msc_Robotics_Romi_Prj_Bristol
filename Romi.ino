/* ROMI PROJECT
   University Of Bristol
   Robotics System
   Furkan Cam
   Sn: 2079193
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

#define M_SPEED 20

/* Romi tasks */
ROMI_TASKS romi_task {FIRST_STARTING_TASK};

/* Motor instances */
myMotor<uint8_t> leftMotorInstance(L_DIR_PIN, L_PWM_PIN);
myMotor<uint8_t> rightMotorInstance(R_DIR_PIN, R_PWM_PIN);
/* Line sensor instance */
lineSensor<uint8_t> lineSensorIns;

//Motor speed variables while lineSensing
sensorMotorPowers sMPower;
//PID instances for right and left motor
PID pidForLeft(KP_VAL, KI_VAL, KD_VAL);
PID pidForRight(KP_VAL, KI_VAL, KD_VAL);

//These variables for handling the finite state machine
uint8_t current_state;
uint8_t next_state = IDLE_STATE;
uint32_t blocking_time {0};

int left_motor_speed {M_SPEED};
int right_motor_speed {M_SPEED};
int turning_angle {0};
/* Implemented Tasks */

/* Define all used instances for non-blocking millis functions.
   First parameter is the function name which is intented to call.
   The second parameter is the period of the function in in millisecond.
   We will use two non-blocking function for handling our tasks, one of them
   will implement the line following task, while other one will be
   resposible for the motor driving,and the former and latter will implement
   every their tasks in every 15ms.
*/
void lineSensingTask(void);
taskInsert lineSensingTaskIns(lineSensingTask, 45);

void motorHandleTask(void);
taskInsert motorHandleTaskIns(motorHandleTask, 15);

//Start Romi from origin point
kinematics knm(0, 0);

static float target_angle = {0};
static float angle_start_point = {0};

//Count the turning number
uint16_t count_turning {0};

float home_distance = {0};

void lineSensingTask(void) {
  /* if the obtained values are higher than
     the determined threshold, we are on line
     and we should implement our motor speed
     according to this situation.
  */
  if (lineSensorIns.isLeftOnline() || lineSensorIns.isRightOnline()) {
    //if our robot on the right or left of the line, set it on the midst
    lineSensorIns.calculateMotorSpeed(sMPower);
    
    leftMotorInstance.motorControl(pidForLeft.updateValue(M_SPEED - sMPower.left_motor_power, leftMotorInstance.readMotorSpeed(&count_e0)) / 3.0);
    rightMotorInstance.motorControl(pidForRight.updateValue(M_SPEED - sMPower.right_motor_power, rightMotorInstance.readMotorSpeed(&count_e1)) / 3.0);

  }
}

void motorHandleTask() {
  leftMotorInstance.motorControl(pidForLeft.updateValue(left_motor_speed, leftMotorInstance.readMotorSpeed(&count_e0)));
  rightMotorInstance.motorControl(pidForRight.updateValue(right_motor_speed, rightMotorInstance.readMotorSpeed(&count_e1)));
}

/* this function is designed to understand which
   task is handling. */
void getRomiTask() {
  if (count_turning == 3) {
    if ( turning_angle == 90) {
      romi_task = FIRST_STARTING_TASK;
    } else if (turning_angle == -90) {
      romi_task = SECOND_STARTING_TASK;
    }

  }
}

void setup() {
  //Start board support packege which is spesific for our mcu
  bsp_ctor();
  //Assign default for our sensor
  lineSensorIns.setTreshold(LINE_TRESHOLD);
  //Reset all PID variables
  pidForRight.reset();
  pidForLeft.reset();

  //get 1000 sample and calibrate the line sensors
  lineSensorIns.calibrate(1000);

  //The sound simulation
  for (int i = 0; i < 10; i++) {
    tone(BUZZER, 850); // Send 850Hz sound signal...
    delay(i * 10);
    noTone(BUZZER);     // Stop sound...
    delay(i * 10);
  }

  delay(1000);

  GO_HANDLE(IDLE_STATE); // start with handling IDLE state
}

void loop() {
  /* This static function will be called for updating
     the time. It will be the same for all instances.
  */
  taskInsert::executeTasks();

  // Update the kinematic variables using the encoder counting rates.
  knm.kinematicupdate(count_e0, count_e1);

  switch (current_state) {
    case IDLE_STATE: {

        GO_HANDLE(READ_LINE_SENSOR);
        break;
      }

    case READ_LINE_SENSOR: {
        if (lineSensorIns.isOnLine()) {
          /* If robot find a line go to the other state
             up to the conditions */
          Serial.println("online!!!");

          /* to count a distance from a point we should reset the
             beginning distance values.
          */
          knm.resetDistanceFrom();

          //if our romi encounter a gap go settling state
          if (romi_task == FIRST_STARTING_TASK) {
            if (count_turning == 7) {
              /* Romi will encounter the gap, to be able to pass the gap
                 we need to find a new line just like at the begining state
              */
              GO_HANDLE(SETTLING_LINE_STATE);
            } else {
              GO_HANDLE(ON_LINE_STATE);
            }
          } else if (romi_task == SECOND_STARTING_TASK) {
            if (count_turning == 12) {
              GO_HANDLE(SETTLING_LINE_STATE);
            } else {
              GO_HANDLE(ON_LINE_STATE);
            }
          } else {
            GO_HANDLE(ON_LINE_STATE);
          }
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
          /* If Robot enters the on-line state
             wait until its overpassing
          */
          GO_HANDLE(ON_LINE_STATE);

        } else {
          /* Robot overpassed the line, we need to find
             new line turning 90 degree around it. However,
             firstly let robot go little further (10mm) to settle
             correctly before turning.
          */
          if (knm.getDistanceFrom() > 10) {
            //After getting 10mm further stop the robot.
            leftMotorInstance.motorControl(0);
            rightMotorInstance.motorControl(0);
            if (romi_task == FIRST_STARTING_TASK) {
              /*If we are in the first Starting task
                there will be unique circumstances that
                we need to implement. */
              if (count_turning == 10) {
                // If we are on our 10th turning turn left instead right
                turning_angle = -90;
              } else {
                // In other cases, turn right for the first checking of line
                turning_angle = 90;
              }
            }
            GO_HANDLE(FIND_LINE);
          }
        }
        break;
      }

    case FIND_LINE: {
        Serial.println("find line-state");
        //This macro is used for waiting in a non-blocking way.
        WAIT_NONBLOCKING_SANE_MS(500, FIND_LINE);
        angle_start_point = knm.get_angle();
        target_angle = angle_start_point + turning_angle;
        GO_HANDLE(BUZZER_STATE);
        break;
      }

    case BUZZER_STATE: {
        //Give a bip before every turning state.
        tone(BUZZER, 850); // Send 1KHz sound signal...
        WAIT_NONBLOCKING_SANE_MS(50, BUZZER_STATE);
        noTone(BUZZER);     // Stop sound...
        GO_HANDLE(TURN_ROMI_STATE);
        break;
      }

    // TURN ROMI STATE
    case TURN_ROMI_STATE: {
        /*this sate will handle the romi turning process according to
          value of turning_angle, if it is 90 romi will turn 90 degree right
          if it is -180, romi will turn left 180 degree.*/
        knm.printVals();
        float system_angle = knm.get_angle();

        if (((turning_angle > 0) && (system_angle > 0)) || ((turning_angle > 0) && (system_angle < 0))) {
          if (system_angle < target_angle)
          {
            Serial.println("state1............................ ");
            left_motor_speed = M_SPEED;
            right_motor_speed = -M_SPEED;
          } else {
            leftMotorInstance.motorControl(0);
            rightMotorInstance.motorControl(0);
            count_turning++;
            //Understand which task is handled now
            if ( count_turning == 3)
              getRomiTask();
            BREAK_AND_GO(CHECK_LINE_STATE);
          }
        } else if (((turning_angle < 0) && (system_angle > 0)) || ((turning_angle < 0) && (system_angle < 0))) {
          if (system_angle > target_angle)
          {
            Serial.println("state2............................ ");
            left_motor_speed = -M_SPEED;
            right_motor_speed = M_SPEED;
          } else {
            leftMotorInstance.motorControl(0);
            rightMotorInstance.motorControl(0);
            count_turning++;
            //Understand which task is handled now
            if ( count_turning == 3)
              getRomiTask();
            BREAK_AND_GO(CHECK_LINE_STATE);
          }
        }

        motorHandleTaskIns.callMyTask();

        GO_HANDLE(TURN_ROMI_STATE);
        break;
      }

    case CHECK_LINE_STATE: {
        if (lineSensorIns.isOnLine()) {
          //If we are on the line, settle the line
          WAIT_NONBLOCKING_SANE_MS(500, CHECK_LINE_STATE);
          GO_HANDLE(SETTLING_LINE_STATE);
        } else {
          /* If we are not on the line, we should look around
             whether there is a suitable line for us. There will
             be also different cases which should be considered
             according to count_turning rate.
          */
          if (romi_task == FIRST_STARTING_TASK) {
            if (count_turning >= 5 && count_turning < 7 || count_turning >= 10) {
              if (count_turning == 5) {
                turning_angle = 90;
                GO_HANDLE(FIND_LINE);
              } else if (count_turning == 6) {
                count_turning++;
                GO_HANDLE(READ_LINE_SENSOR);
              } else if (count_turning >= 10) {
                knm.resetDistanceFrom();
                home_distance = knm.homeDistance();
                GO_HANDLE(RETURN_HOME);
              }
            }  else {
              turning_angle = -90;
              GO_HANDLE(FIND_LINE);
            }
          } else if (romi_task == SECOND_STARTING_TASK) {
            if ( count_turning == 4) {
              turning_angle = 90;
              GO_HANDLE(FIND_LINE);
            } else if ( count_turning == 5) {
              turning_angle = 180;
              GO_HANDLE(FIND_LINE);
            } else if ( count_turning == 10) {
              turning_angle = 90;
              GO_HANDLE(FIND_LINE);
            } else if ( count_turning == 11) {
              count_turning++;
              GO_HANDLE(READ_LINE_SENSOR);
            } else if (count_turning == 16) {
              knm.resetDistanceFrom();
              home_distance = knm.homeDistance();
              GO_HANDLE(RETURN_HOME);
            }
            else {
              turning_angle = -90;
              GO_HANDLE(FIND_LINE);
            }
          }

        }
        break;
      }

    case SETTLING_LINE_STATE: {
        //This state is to getting the line until it finishes.
        Serial.println("settling line-state");
        if (lineSensorIns.isOnLine()) {
          //Call the line following non-blocking task
          lineSensingTaskIns.callMyTask();
          GO_HANDLE(SETTLING_LINE_STATE);
        } else {
          /* if there is no line, stop the motor, and search for
             another one. */
          leftMotorInstance.motorControl(0);
          rightMotorInstance.motorControl(0);
          knm.resetDistanceFrom();
          turning_angle = 90;
          GO_HANDLE(FOLLOW_LINE_STATE);
        }
        break;
      }

    case FOLLOW_LINE_STATE: {
        //To be able to go further 10mm, we are adjusting the motor speed rate.
        Serial.println("FOLLOW_LINE_STATE-state");
        WAIT_NONBLOCKING_SANE_MS(100, FOLLOW_LINE_STATE);
        leftMotorInstance.motorControl(-15);
        rightMotorInstance.motorControl(-15);
        GO_HANDLE(ON_LINE_STATE);
        break;
      }

    case RETURN_HOME : {
        /*If the romi finihes its task succesfully, it should go its initial point
          To calculate the initial point's distance, we are using the distance from function.*/
        Serial.print("home Distance: "); Serial.print(home_distance);
        Serial.print(" - distance from: "); Serial.print(knm.getDistanceFrom());
        if ((home_distance ) > knm.getDistanceFrom()) {
          left_motor_speed = M_SPEED;
          right_motor_speed = M_SPEED;
          motorHandleTaskIns.callMyTask();
          GO_HANDLE(RETURN_HOME);
        } else {
          GO_HANDLE(STOP_MOTOR_STATE);
        }
        break;
      }


    case STOP_MOTOR_STATE: {
        //After all implementation, Romi will come here and stop the motors.
        leftMotorInstance.motorControl(0);
        rightMotorInstance.motorControl(0);
        GO_HANDLE(STOP_MOTOR_STATE);
        break;
      }

    case NON_BLOCKING_DELAY_STATE: {
        //This state is designed for the non blocking waiting
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


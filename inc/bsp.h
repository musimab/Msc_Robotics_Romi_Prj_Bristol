#ifndef _BSP_H_
#define _BSP_H_

/* Motor pins definitions */
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15

#define MAX_MOTOR_POWER 255

#define LED_1 6
#define LED_2 7
#define LED_3 8
#define outputA 2 // interrupt pin
#define outputB 10

enum {
  STOP_MOTORS,
  MOVE_FORWARD,
  MOVE_BACKWARD,
  TURN_RIGHT,
  TURN_LEFT,
};

/* These definitions are about the encoder */
extern volatile int counter;
extern volatile int aState;
extern volatile int aLastState;

template<typename MSpeed>
void motorControl(uint8_t command , MSpeed motorSpeed)
{
  switch (command) {
    case STOP_MOTORS: {
        digitalWrite(R_DIR_PIN, LOW);
        analogWrite(R_PWM_PIN, 0);
        digitalWrite(L_DIR_PIN, LOW);
        analogWrite(L_PWM_PIN, 0);
        break;
      }

    case MOVE_FORWARD: {
        digitalWrite(R_DIR_PIN, HIGH);
        analogWrite(R_PWM_PIN, motorSpeed);
        digitalWrite(L_DIR_PIN, HIGH);
        analogWrite(L_PWM_PIN, motorSpeed);
        break;
      }

    case MOVE_BACKWARD: {
        digitalWrite(R_DIR_PIN, HIGH);
        analogWrite(R_PWM_PIN, -motorSpeed);
        digitalWrite(L_DIR_PIN, HIGH);
        analogWrite(L_PWM_PIN, -motorSpeed);
        break;
      }

    case TURN_RIGHT: {
        digitalWrite(R_DIR_PIN, HIGH);
        analogWrite(R_PWM_PIN, -motorSpeed);
        digitalWrite(L_DIR_PIN, HIGH);
        analogWrite(L_PWM_PIN, motorSpeed);
        break;
      }

    case TURN_LEFT: {
        digitalWrite(R_DIR_PIN, HIGH);
        analogWrite(R_PWM_PIN, motorSpeed);
        digitalWrite(L_DIR_PIN, HIGH);
        analogWrite(L_PWM_PIN, -motorSpeed);
        break;
      }

    default : {
        command = STOP_MOTORS;
        break;
      }

  }
}

boolean nonBlockingDelay(unsigned long dly);
float readMotorSpeedTask(void);
void smartMotorControl(int r_motor_speed, int l_motor_speed);
void bsp_ctor(void);
void encoderInterrupt();

#endif _BSP_H_

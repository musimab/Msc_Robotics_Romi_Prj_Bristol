#ifndef _BSP_H_
#define _BSP_H_

#define MAX_MOTOR_SPEED 100

#define LED_1 6
#define LED_2 7
#define LED_3 8

enum {
  STOP_MOTORS,
  MOVE_FORWARD,
  MOVE_BACKWARD,
  TURN_RIGHT,
  TURN_LEFT,
};

/* These definitions are about the encoder */
extern volatile long count_e1; // used by encoder to count the rotation
extern volatile long count_e0; // used by encoder to count the rotation
extern volatile int aState;
extern volatile int aLastState;

boolean nonBlockingDelay(unsigned long dly);
float readMotorSpeedTask(void);
void smartMotorControl(int r_motor_speed, int l_motor_speed);
void bsp_ctor(void);
void encoderInterrupt();
void setupEncoder0();

#endif _BSP_H_

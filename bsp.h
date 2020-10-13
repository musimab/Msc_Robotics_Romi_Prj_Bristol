#ifndef _BSP_H_
#define _BSP_H_

#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15
#define LED_1 6
#define LED_2 7
#define LED_3 8
#define outputA 2 // interrupt pin
#define outputB 10

extern volatile int counter;
extern volatile int aState;
extern volatile int aLastState;

void bsp_ctor(void);
void encoderInterrupt();

#endif _BSP_H_

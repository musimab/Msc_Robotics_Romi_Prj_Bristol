/* 
 *	Furkan Cam 
 *
 *  Board suppor package cpp
 *  
 *	Board spesific implementations
 */

#include "Arduino.h"
#include "..\inc\bsp.h"
#include "..\inc\lineSensor.hpp"

#define E1_A_PIN  7
#define E1_B_PIN  23
#define E0_A_PIN  26

#define CW  -=
#define CCW +=

//Left motor includes encoder 0
//Right motor includes encoder 1
volatile long count_e1 = 0;
volatile long count_e0 = 0;
volatile int aState;
volatile int aLastState;

void bsp_ctor(void) {
  Serial.begin(9600);
  pinMode(BUZZER, OUTPUT); // Set buzzer - pin 9 as an output 
  /* interrupt configurations */
  attachInterrupt(digitalPinToInterrupt(E1_A_PIN), encoderInterrupt, CHANGE);
  setupEncoder0();
    
  Serial.println(" ***Reset*** ");
}

void setupEncoder0() {

  // Initialise our count value to 0.
  // Initialise the prior A & B signals
  // to zero, we don't know what they were.

  // Setting up E0_PIN_B:
  DDRE = DDRE & ~(1 << DDE6);
  //DDRE = DDRE & B11111011; // Same as above.

  // We need to enable the pull up resistor for the pin
  // To do this, once a pin is set to input (as above)
  // You write a 1 to the bit in the output register
  PORTE = PORTE | (1 << PORTE2 );
  //PORTE = PORTE | 0B00000100;

  // Encoder0 uses conventional pin 26
  pinMode( E0_A_PIN, INPUT );
  digitalWrite( E0_A_PIN, HIGH ); // Encoder 0 xor

  // Disable interrupt first
  PCICR = PCICR & ~( 1 << PCIE0 );
  // PCICR &= B11111110;  // Same as above

  // 11.1.7 Pin Change Mask Register 0 – PCMSK0
  PCMSK0 |= (1 << PCINT4);

  // Page 91, 11.1.6 Pin Change Interrupt Flag Register – PCIFR
  PCIFR |= (1 << PCIF0);  // Clear its interrupt flag by writing a 1.

  // Enable
  PCICR |= (1 << PCIE0);
}

/* this interrupt will count rotate rate of encoder */
void encoderInterrupt() {
	
  static boolean oldE1_A = false ;
  static boolean oldE1_B = false ;
  	
  boolean newE1_B = digitalRead( E1_B_PIN );
  boolean newE1_A = digitalRead( E1_A_PIN );
  
  // Some clever electronics combines the
  // signals and this XOR restores the
  // true value.
  newE1_A ^= newE1_B;
  
    // State :  (bit3)  (bit2)  (bit1)  (bit0)
  // State :  New A,  New B,  Old A,  Old B.
  byte state = 0;
  state = state | ( newE1_A  << 3 );
  state = state | ( newE1_B  << 2 );
  state = state | ( oldE1_A  << 1 );
  state = state | ( oldE1_B  << 0 );
  
  // This is an inefficient way of determining
  // the direction.  However it illustrates well
  // against the lecture slides.
  // CW -> clock wise CCW -> counter clock wise
  if( state == 1 ) {          
    count_e1 CW 1;             
  } else if( state == 2 ) {    
	count_e1 CCW 1;	
  } else if( state == 4 ) {    
	count_e1 CCW 1;	
  } else if( state == 7 ) {
	count_e1 CW 1;
  } else if( state == 8 ) {
	count_e1 CW 1;
  } else if( state == 11) {
  	count_e1 CCW 1;
  } else if( state == 13) {
  	count_e1 CCW 1;
  } else if( state == 14) {
  	count_e1 CW 1;
  }

  // Save current state as old state for next call.
  oldE1_A = newE1_A;
  oldE1_B = newE1_B;
}
/* this interrupt will count rotate rate of encoder */
ISR( PCINT0_vect ) {

  static boolean oldE0_A = false ;
  static boolean oldE0_B = false ;

  boolean newE0_B = PINE & (1<<PINE2);
  //boolean newE0_B = PINE & B00000100;  // Does same as above.

  // Standard read fro the other pin.
  boolean newE0_A = digitalRead( E0_A_PIN ); // 26 the same as A8

  // Some clever electronics combines the
  // signals and this XOR restores the 
  // true value.
  newE0_A ^= newE0_B;

  // Create a bitwise representation of our states
  // We do this by shifting the boolean value up by
  // the appropriate number of bits, as per our table
  // header:
  //
  // State :  (bit3)  (bit2)  (bit1)  (bit0)
  // State :  New A,  New B,  Old A,  Old B.
  byte state = 0;                   
  state = state | ( newE0_A  << 3 );
  state = state | ( newE0_B  << 2 );
  state = state | ( oldE0_A  << 1 );
  state = state | ( oldE0_B  << 0 );


  // This is an inefficient way of determining
  // the direction.  However it illustrates well
  // against the lecture slides.
  // CW -> clock wise CCW -> counter clock wise
  if( state == 1 ) {          
    count_e0 CW 1;             
  } else if( state == 2 ) {    
	count_e0 CCW 1;	
  } else if( state == 4 ) {    
	count_e0 CCW 1;	
  } else if( state == 7 ) {
	count_e0 CW 1;
  } else if( state == 8 ) {
	count_e0 CW 1;
  } else if( state == 11) {
  	count_e0 CCW 1;
  } else if( state == 13) {
  	count_e0 CCW 1;
  } else if( state == 14) {
  	count_e0 CW 1;
  }
  // Save current state as old state for next call.
  oldE0_A = newE0_A;
  oldE0_B = newE0_B; 
}

boolean nonBlockingDelay(unsigned long dly) {

    static unsigned long delay_last_timestamp = 0;
    static boolean is_entered_first = true;

    if (is_entered_first) {
        delay_last_timestamp = millis();
        is_entered_first = false;
    }
    
    unsigned long delay_time_now = millis();
    if ((delay_time_now - delay_last_timestamp) > dly)
    {
        is_entered_first = true;
        return true;
    }
    else {
        return false;
    }
    
}



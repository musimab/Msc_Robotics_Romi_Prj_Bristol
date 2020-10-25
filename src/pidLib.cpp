
#include "Arduino.h"
#include "C:\Users\Furkan\Desktop\Bristol\RoboticsSystem\RoboticsSystemRomi\Romi\inc\pidLib.h"
#include "..\inc\bsp.h"

/*
   Class constructor
   This runs whenever we create an instance of the class
*/
PID::PID(float P, float I, float D)
{
	//Store the gains
	setGains(P, I, D);
	//Set last_millis
	reset();
}

/*
   This function prints the individual contributions to the total contol signal
   You can call this yourself for debugging purposes, or set the debug flag to true to have it called
   whenever the update function is called.
*/
void PID::printComponents() {


}


void PID::reset() {
	last_error = 0;
	integral_error = 0;
	Kp_output = 0;
	Ki_output = 0;
	Kd_output = 0;
	//last_millis     = millis();
}

/*
   This function sets the gains of the PID controller
*/
void PID::setGains(float P, float I, float D) {
	Kp = P;
	Kd = D;
	Ki = I;
}

/*
   This is the update function.
   This function should be called repeatedly.
   It takes a measurement of a particular quantity and a desired value for that quantity as input
   It returns an output; this can be sent directly to the motors,
   or perhaps combined with other control outputs
*/
float PID::updateValue(float demand, float measurement) {

	float error = measurement - demand;
	float err_diff = error - last_error;
	
	integral_error += error;

	if (integral_error > 100)
    	integral_error = 100;
  	else if (integral_error < -100)
    	integral_error = -100;
    	
	//Calculate P,I,D Term contributions.
	Kp_output = Kp * error;
	Kd_output = Kd * err_diff; 
	Ki_output = Ki * integral_error; 

	//Add the three components to get the total output 
	//output signal -> total pid output value
	output_signal = Kp_output + Kd_output + Ki_output;

	if(output_signal > 125)
	   output_signal = 125;
	else if(output_signal < -125)
		output_signal = -125;

	last_error = error;
	// Pass the result back.
	return output_signal;
}

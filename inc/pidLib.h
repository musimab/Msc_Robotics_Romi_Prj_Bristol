#ifndef _PID_H_
#define _PID_H_

class PID {
  public:

    PID(float P, float I, float D);                 // Constructor, not order of P I & D arguments when calling.
    void setGains(float P, float I, float D);       // This function updates the values of the gains
    void reset();                                   // Useful to remove any intergral wind-up
    float updateValue(float demand, float measurement);  // This update takes a demand and measurement.

    void printComponents(); //This function prints the individual components of the control signal and can be used for debugging

    float Kp_output = 0;
    float Ki_output = 0;
    float Kd_output = 0;

    /* Private functions and variables are defined here. These functions / variables cannot be accessed from outside the class.
       For example, if we try to set the value of Kp in the file "Romi.h", we will get an error (Try it out!)
    */
  private:

    //Control gains
    float Kp; //Proportional
    float Ki; //Integral
    float Kd; //Derivative


    //Values to store
    float output_signal       = 0;
    float last_error          = 0; //For calculating the derivative term
    float integral_error      = 0; //For storing the integral of the error

};

#endif _PID_H_

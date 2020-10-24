#ifndef _LINESENSOR_H
#define _LINESENSOR_H

#include "bsp.h"

#define LINE_TRESHOLD 200

#define LINE_LEFT_PIN A4
#define LINE_CENTRE_PIN A3
#define LINE_RIGHT_PIN A2

struct sensorMotorPowers {
  float left_motor_power;
  float right_motor_power;
};

template <typename S_Pin>
class lineSensor {

  private:
    S_Pin left_pin {0};
    S_Pin centre_pin {0};
    S_Pin right_pin {0};

    float cal_l_value {0.0};
    float cal_c_value {0.0};
    float cal_r_value {0.0};

    float line_treshold{ LINE_TRESHOLD };

  public:

    float left_sensor_val{ 0.0 };
    float centre_sensor_val{ 0.0 };
    float right_sensor_val{ 0.0 };

    // default Constructor
    lineSensor() {
      left_pin = LINE_LEFT_PIN;
      centre_pin = LINE_CENTRE_PIN;
      right_pin = LINE_RIGHT_PIN;

      pinMode( LINE_LEFT_PIN, INPUT );
      pinMode( LINE_CENTRE_PIN, INPUT );
      pinMode( LINE_RIGHT_PIN, INPUT );
    }

    // constructer with initializer lists
    lineSensor( S_Pin _left_pin, S_Pin _centre_pin, S_Pin _right_pin):
      left_pin {_left_pin}, centre_pin{_centre_pin}, right_pin{_right_pin} {
      pinMode( left_pin, INPUT );
      pinMode( centre_pin, INPUT );
      pinMode( right_pin, INPUT );
    }

    /* Bring the robot over the white surface and
       wait until the system calibrate it.
    */
    void calibrate(uint16_t cal_rate = 50) {
      float l_value{0}, c_value {0}, r_value {0};
	  Serial.println("calibrating");
      for (int i = cal_rate; i >= 0; i--) {
        l_value += analogRead(left_pin);
        c_value += analogRead(centre_pin);
        r_value += analogRead(right_pin);
        Serial.print(".");
      }
      cal_l_value = l_value / (float)cal_rate;
      cal_c_value = c_value / (float)cal_rate;
      cal_r_value = r_value / (float)cal_rate;
    }

    void readLeftCalibratedLineVal(void) {
      left_sensor_val = analogRead(left_pin);
      left_sensor_val -= cal_l_value;
    }

    void readRightCalibratedLineVal(void) {
      centre_sensor_val = analogRead(centre_pin);
      centre_sensor_val -= cal_c_value;
    }

    void readCentreCalibratedLineVal(void) {
      right_sensor_val = analogRead(right_pin);
      right_sensor_val -= cal_r_value;
    }

    void readCalibratedLineVals(void) {
      // this function will calculate calibrated values
      readLeftCalibratedLineVal();
      readRightCalibratedLineVal();
      readCentreCalibratedLineVal();
    }

    void calculateMotorSpeed(sensorMotorPowers & myPowers) {
      readCalibratedLineVals();
      float total_val = left_sensor_val + centre_sensor_val + right_sensor_val;
      myPowers.left_motor_power  = MAX_MOTOR_SPEED * ((left_sensor_val - right_sensor_val) / total_val);
      myPowers.right_motor_power = -myPowers.left_motor_power;
    }

    void setTreshold(float threshold) {
        line_treshold = threshold;
    }

    boolean isLeftOnline() {
        readLeftCalibratedLineVal();
        return (left_sensor_val > line_treshold);
    }

    boolean isCentreOnline() {
        readCentreCalibratedLineVal();
        return (centre_sensor_val > line_treshold);
    }

    boolean isRightOnline() {
        readRightCalibratedLineVal();
        return (right_sensor_val > line_treshold);
    }

    // Write a routine here to check if your
    // sensor is on a line (true or false).
    boolean isOnLine() {
      readCalibratedLineVals();
      if (isLeftOnline() || isCentreOnline() || isRightOnline()) {
        return true;
      }
      return false;
    }
};

#endif _LINESENSOR_H

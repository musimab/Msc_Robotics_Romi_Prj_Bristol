#ifndef _LINESENSOR_H
#define _LINESENSOR_H

#define POWER_MAX 255
#define LINE_LEFT_PIN A2
#define LINE_CENTRE_PIN A3
#define LINE_RIGHT_PIN A4

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

    float left_sensor_val  {0.0};
    float centre_sensor_val{0.0};
    float right_sensor_val {0.0};

  public:
    // default Constructor
    lineSensor() {
      S_Pin left_pin = LINE_LEFT_PIN;
      S_Pin centre_pin = LINE_CENTRE_PIN;
      S_Pin right_pin = LINE_RIGHT_PIN;

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

      for (uint16_t i = cal_rate; i >= 0; i--) {
        l_value += analogRead(left_pin);
        c_value += analogRead(centre_pin);
        r_value += analogRead(right_pin);
      }
      cal_l_value = l_value / (float)cal_rate;
      cal_c_value = c_value / (float)cal_rate;
      cal_r_value = r_value / (float)cal_rate;
    }

    void readLine(void) {
      // this function will calculate calibrated values
      left_sensor_val = analogRead(left_pin);
      left_sensor_val -= cal_l_value;
      centre_sensor_val = analogRead(centre_pin);
      centre_sensor_val -= cal_c_value;
      right_sensor_val = analogRead(right_pin);
      right_sensor_val -= cal_r_value;
    }

    void calculateMotorSpeed(sensorMotorPowers & myPowers) {
      readLine();
      float total_val = left_sensor_val + centre_sensor_val + right_sensor_val;
      myPowers.left_motor_power  = POWER_MAX * ((left_sensor_val - right_sensor_val) / total_val);
      myPowers.right_motor_power = -myPowers.left_motor_power;
    }

    // Write a routine here to check if your
    // sensor is on a line (true or false).
    boolean onLine( float threshold ) {
      readLine();
      if (left_sensor_val > threshold || centre_sensor_val > threshold
          || right_sensor_val > threshold ) {
        return true;
      }
      return false;
    }
};

#endif _LINESENSOR_H

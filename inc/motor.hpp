#ifndef _MOTOR_H_
#define _MOTOR_H_

/* Motor pins definitions */
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15

template <typename motorType>
class myMotor {
private:
	
	motorType motor_pwm_pin {0};
	motorType motor_dir_pin {0};
	unsigned long mSpeed_last_timestamp = millis();
	int last_counted_val = 0;
	
public:
	
	myMotor() {
	}
	
	myMotor(uint8_t _motor_dir_pin, uint8_t _motor_pwm_pin) : 
	motor_dir_pin {_motor_dir_pin}, motor_pwm_pin {_motor_pwm_pin} {
	    /* Motor pin configuration */
  		pinMode( motor_dir_pin, OUTPUT );
  		pinMode( motor_pwm_pin, OUTPUT );
	}
	
	void motorControl(int motor_power)
	{
		if (abs(motor_power) > 100 ) {
			//Serial.print("non-valid values have been entered!");
			return;
		}
		if (motor_power > 0) {
			digitalWrite(motor_dir_pin, HIGH);
			analogWrite(motor_pwm_pin, motor_power);
		}
		else if (motor_power < 0) {
			digitalWrite(motor_dir_pin, LOW);
			analogWrite(motor_pwm_pin, -motor_power);
		}else if (motor_power == 0) {
			digitalWrite(motor_dir_pin, HIGH);
			analogWrite(motor_pwm_pin, motor_power);
		}
	}
	
	float readMotorSpeed(const long * const encoder_counter) {
    /* In this task we are getting our current
       motor speed.
    */
    unsigned long mSpeed_time_now = millis();

    float counterDiff = *encoder_counter - last_counted_val;
    float velocity = counterDiff / (float)(mSpeed_time_now - mSpeed_last_timestamp);
    velocity = MAX_MOTOR_SPEED * velocity / 3.0 /* 3.0 max speed for 100 */;
    last_counted_val = *encoder_counter;
    mSpeed_last_timestamp = millis();
    return velocity;
	}

};


#endif _MOTOR_H_

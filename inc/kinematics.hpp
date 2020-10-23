#ifndef _KINEMATICS_H
#define _KINEMATICS_H

#include "math.h"

#define WHEEL_CIRFUM  219.911485
#define MM_PER_COUNT  0.15271631 // 30
#define WHEEL_PERREV  1440
#define MAGNET_PERREV 12
#define ANGLE_PER_CNT 0.25
#define PULSE_FOR_ROMI 8

class kinematics {

  private:

    float Ynew {0}, Xnew{0}; // Y - coordinate
	float angle_change{0};
    long last_enc0 {0}, last_enc1 {0}, enc_change0 {0}, enc_change1 {0};

  public:
    // What variables do you need?
    // What is the appropriate type?

    kinematics() {
      // ...
    }   // end of constructor.

    // Routine to execute the update to
    // kinematics
    
    float get_Yaxis() {
    	return Ynew;
	}
	
	float get_angle(long encoder_c0, long encoder_c1 ){
		angle_change += (encoder_c0 - encoder_c1) * ANGLE_PER_CNT ;
		if(abs(angle_change) >= 360)
		   angle_change = 0;
		return angle_change;
		
	}
    
    void kinematicupdate(long encoder_c0, long encoder_c1) {
    	
      float distance = MM_PER_COUNT * abs(encoder_c0 - encoder_c1);

      //Ynew += distance * sin() ;
      //Xnew += distance * cos() ;

      // update last encoder values with new ones
      last_enc0 = encoder_c0;
      last_enc1 = encoder_c1;

    }

}; // End of class definition.





#endif _KINEMATICS_H

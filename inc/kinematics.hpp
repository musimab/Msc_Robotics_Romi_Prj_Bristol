#ifndef _KINEMATICS_H
#define _KINEMATICS_H

#include "math.h"

#define PI            3.14159265
#define DETORA        0.017453292 // deg to rad coefficient
#define MM_PER_COUNT  0.20 // 30
#define ANGLE_ERROR   0.0132405 //0.1365
#define DIS_TO_ANG    0.1486 
#define ROTATE_VAL(a) a * (1 + ANGLE_ERROR)
#define DEG_TO_RAD(d) d*DETORA
#define CORR_RATE     1.85 //correctness coefficient

class kinematics {

  private:

    float Xnew{0}, Ynew {0}; // Y - coordinate
    long last_enc0 {0}, last_enc1 {0};
	float distance {0}, angle{0}, last_distance{0};
	float dis_diff {0}, distanceFrom{0};
  public:
    // What variables do you need?
    // What is the appropriate type?

    kinematics(float x_coordinate, float y_coordinate) : 
	Xnew{x_coordinate}, Ynew{y_coordinate} {
    	
    	
    }   // end of constructor.

    // Routine to execute the update to
    // kinematics
    
    float get_Yaxis() {
    	return Ynew;
	}
	
	float get_angle(){
      	return (angle);
	}
    
    //To measure two distance, before reset the value
    void resetDistanceFrom() {
    	distanceFrom = 0;
	}
	
    //Measure between two distance
    float getDistanceFrom() {
     distanceFrom += dis_diff;
     return distanceFrom;
	}
    
    float homeDistance () {
    	return sqrt((Ynew*Ynew) + (Xnew * Xnew)) * CORR_RATE;
	}
    
    void printVals() {
    	Serial.print("distance: "); Serial.print(distance);
    	Serial.print(" angle: "); Serial.print(angle);
    	Serial.print(" X: "); Serial.print(Xnew);
    	Serial.print(" Y: "); Serial.println(Ynew);
	}
    
    void kinematicupdate(volatile long & encoder_c0, volatile  long & encoder_c1) {
      float count_diff_e0 = encoder_c0 - last_enc0;
      float count_diff_e1 = encoder_c1 - last_enc1;
      
      distance += MM_PER_COUNT * (count_diff_e0 + count_diff_e1) /2.0;
      angle +=  DIS_TO_ANG * ((count_diff_e0 - count_diff_e1) /2.0) ; 
		
	  dis_diff = distance-last_distance;	
	  
      Ynew += (dis_diff) * sin(DEG_TO_RAD(angle));
      Xnew += (dis_diff) * cos(DEG_TO_RAD(angle));

      // update last encoder values with new ones
      last_enc0 = encoder_c0;
      last_enc1 = encoder_c1;
      last_distance = distance;

    }

}; // End of class definition.





#endif _KINEMATICS_H

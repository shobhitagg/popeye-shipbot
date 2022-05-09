#ifndef __DCMOTOR_H_
#define __DCMOTOR_H_

#include "Arduino.h"

class DCMotor
{
	public:
		int encaPin, encbPin, pwmPin, dirPin;
		int orientation;
		int pos;
		float vel;
		float velTarget;
		int maxPwr, minPwr;
		
		DCMotor(int arg1, int arg2, int arg3, int arg4, int arg5=1){
			maxPwr = 255;
			minPwr = 0;
			
			encaPin = arg1;
			encbPin = arg2;
			pwmPin = arg3;
			dirPin = arg4;
			orientation = arg5;
		}
		float positionPID(int target);
		void setMotor(float pwr);
		void velUpdate();
};

#endif

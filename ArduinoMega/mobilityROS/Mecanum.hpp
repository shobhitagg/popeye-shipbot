#ifndef __MECANUM_H_
#define __MECANUM_H_

#include "Arduino.h"
#include "DCMotor.hpp"
#include "UTModule.hpp"

#ifdef __cplusplus
 extern "C" {
#endif

class Mecanum
{
	public:
		DCMotor *M1, *M2, *M3, *M4;
		float lx;
		float ly;
		float wheelRad;
		int X, Y, R, Xdot, Ydot, Rdot;
		int minPwr, maxPwr;
		
		Mecanum(DCMotor arg1, DCMotor arg2, DCMotor arg3, DCMotor arg4, float arg5, float arg6, float arg7){
			M1 = &arg1;
			M2 = &arg2;
			M3 = &arg3;
			M4 = &arg4;			
			lx = arg5;
			ly = arg6;
			wheelRad = arg7;
			minPwr = 0;
			maxPwr = 100;
			
		}
		
		void FKvel(void);
		void IKvel(int, int, int);
		void moveFront(float);
		void moveRight(float);
		void moveTurn(float);
		void stopWheels(void);
		void adjustParallel(UTModule&);
		void alignCorner(UTModule&);
		void alignCorner(UTModule& utModule, char, float right, float front);
		void moveTurn(UTModule& utModule, float);
		void moveRight(UTModule&, char, float);
		void moveFront(UTModule&, float);
		
};

#ifdef __cplusplus
}
#endif

#endif

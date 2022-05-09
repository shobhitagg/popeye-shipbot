#ifndef __UT_MODULE_H_
#define __UT_MODULE_H_

#include "Arduino.h"

class UTModule
{
	public:
		int frTrig, flTrig;
		int frEcho, flEcho;
		int rTrig, lTrig;
		int rEcho, lEcho;
		double utLx;
		
		UTModule(int arg1, int arg2, int arg3, int arg4, double arg5){
			frTrig = arg1;
			flTrig = arg3;
			frEcho = arg2;
			flEcho = arg4;
			utLx = arg5;
		}
		double RLinitialize(int arg1, int arg2, int arg3, int arg4){
			rTrig = arg1;
			lTrig = arg3;
			rEcho = arg2;
			lEcho = arg4;		
		}
		double getThetaError();
		double ultrasonicFR();
		double ultrasonicFL();
		double ultrasonicR();
		double ultrasonicL();
		double ultrasonicF();
    double median(long*, int);
};

#endif

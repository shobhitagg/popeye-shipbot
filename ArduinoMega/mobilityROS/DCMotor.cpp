#include "DCMotor.hpp"

float DCMotor::positionPID(int target)
{
	int posVar;
	static long prevT = 0;
	static float eprev = 0;
	static float eintegral = 0;

	//PID constants
	float kp = 2;
	float kd = 0;
	float ki = 0;
	
	//time difference for limit calculus approx
	long currT = micros();
	float deltaT = ((float)(currT-prevT))/1.0e6;
	prevT = currT;
	
	//error
	int e = pos-target;
	
	//derivative
	float dedt = (e-eprev)/(deltaT);
	
	//integral
	eintegral = eintegral + e*deltaT;
	
	//control signal
	float u = kp*e + kd*dedt + ki*eintegral;
	
	//motor power
	float pwr = fabs(u);
	if(pwr > maxPwr)
		pwr = maxPwr;
	else if(pwr < minPwr)
		pwr = minPwr;   
 
	//motor direction
	if(u<0)
		pwr = -pwr;

	return(pwr);
}

void DCMotor::setMotor(float pwr)
{
	pwr = pwr * orientation;
	if(pwr<0)
		digitalWrite(dirPin, LOW);
	else
		digitalWrite(dirPin, HIGH);
	
	int pwrSend = (int)fabs(pwr);
	
	if(fabs(pwr) < minPwr)
		pwrSend = minPwr;
	else if(fabs(pwr) > maxPwr)
		pwrSend = maxPwr;
	
	analogWrite(pwmPin, pwrSend);
}

void DCMotor::velUpdate()
{
	static int posPrev = 0;
	static long prevT = 0;
	long currT = micros();
	float deltaT = ((float)(currT-prevT))/1.0e6;

	vel = (pos - posPrev)/deltaT;
	
	prevT = currT;
	posPrev = pos;
}

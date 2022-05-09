
#include "Mecanum.hpp"

void Mecanum::FKvel(){
	M1->velUpdate();
	M2->velUpdate();
	M3->velUpdate();
	M4->velUpdate();
	
	Xdot = M1->vel + M2->vel + M3->vel + M4->vel;
	Ydot = -M1->vel + M2->vel + M3->vel - M4->vel;
	Rdot = (-M1->vel + M2->vel - M3->vel + M4->vel)/(lx+ly);
}

void Mecanum::IKvel(int X, int Y, int R){
	M1->velTarget = (X - Y - (lx + ly)*R)/wheelRad;
	M2->velTarget = (X + Y + (lx + ly)*R)/wheelRad;
	M3->velTarget = (X + Y - (lx + ly)*R)/wheelRad;
	M4->velTarget = (X - Y + (lx + ly)*R)/wheelRad;
}

void Mecanum::moveFront(float pwr){
	if(fabs(pwr) < minPwr)
		if(pwr<0)
			pwr = -minPwr;
		else
			pwr = minPwr;
	if(fabs(pwr) > maxPwr)
		if(pwr<0)
			pwr = -maxPwr;
		else
			pwr = maxPwr;

	M1->setMotor(pwr);
	M2->setMotor(pwr);
	M3->setMotor(pwr);
	M4->setMotor(pwr);
}
void Mecanum::moveRight(float pwr){
	if(fabs(pwr) < minPwr)
		if(pwr<0)
			pwr = -minPwr;
		else
			pwr = minPwr;
	if(fabs(pwr) > maxPwr)
		if(pwr<0)
			pwr = -maxPwr;
		else
			pwr = maxPwr;

	M1->setMotor(pwr);
	M2->setMotor(-pwr);
	M3->setMotor(-pwr);
	M4->setMotor(pwr);
}

void Mecanum::moveTurn(float pwr){
	if(fabs(pwr) < minPwr)
		if(pwr<0)
			pwr = -minPwr;
		else
			pwr = minPwr;
	if(fabs(pwr) > maxPwr)
		if(pwr<0)
			pwr = -maxPwr;
		else
			pwr = maxPwr;

	M1->setMotor(pwr);
	M2->setMotor(-pwr);
	M3->setMotor(pwr);
	M4->setMotor(-pwr);
}

void Mecanum::stopWheels(void){
	M1->setMotor(0);
	M2->setMotor(0);
	M3->setMotor(0);
	M4->setMotor(0);
}

void Mecanum::adjustParallel(UTModule& ut)
{
	float thetaError, thetaThreshold, Kp;
	thetaThreshold = 0.02;
	Kp = 300;
  
	thetaError = ut.getThetaError();
  
	while(fabs(thetaError) > thetaThreshold){
		thetaError = ut.getThetaError();
		moveTurn(Kp * thetaError);
	}
  stopWheels();	
}

void Mecanum::alignCorner(UTModule& utModule){

  float dist = 0;
  
  adjustParallel(utModule);
  adjustParallel(utModule);
  delay(500);
  
  if(utModule.ultrasonicR()>15.0){
    while((dist = utModule.ultrasonicR())>15.0){
      moveRight(50);
    }
    stopWheels();
  }
  else{
    while((dist = utModule.ultrasonicR())<15.0){
      moveRight(-50);
    }
    stopWheels();
  }
  adjustParallel(utModule);
  delay(500);
  
  if(utModule.ultrasonicFR()>15.0){
    while((dist = utModule.ultrasonicFR())>15.0){
      moveFront(50);
    }
    stopWheels();
  }
  else{
    while((dist = utModule.ultrasonicFR())<15.0){
      moveFront(-50);
    }
    stopWheels();
  }
  adjustParallel(utModule);
  stopWheels();
  delay(500); 

}


void Mecanum::alignCorner(UTModule& utModule, char sensor, float right, float front){

	float positionThreshold = 1.0;
	float Kp = 10.0;
	
	adjustParallel(utModule); 


	if(sensor == 'r'){
		float cornerRightError = utModule.ultrasonicR() - right;
		while(fabs(cornerRightError) > positionThreshold){
			cornerRightError = utModule.ultrasonicR() - right;
			moveRight(Kp * cornerRightError);
		}	  
	}
	else if(sensor == 'l'){
		float cornerLeftError = utModule.ultrasonicL() - right;
		while(fabs(cornerLeftError) > positionThreshold){
			cornerLeftError = utModule.ultrasonicL() - right;
			moveRight(-Kp * cornerLeftError);
		}	  
	}
	adjustParallel(utModule);
	delay(500);

	float cornerFrontError = (utModule.ultrasonicFR() + utModule.ultrasonicFL())/2 - front;
	while(fabs(cornerFrontError) > positionThreshold){
		cornerFrontError = (utModule.ultrasonicFR() + utModule.ultrasonicFL())/2 - front;
		moveFront(Kp * cornerFrontError);
	}	  
	
	adjustParallel(utModule);
	delay(500);
}

void Mecanum::moveTurn(UTModule& utModule, float value){
	
	float rotSpeed = 18; //deg/seconds at moveTurn(30);
	
	stopWheels();

  if(value>0)
	  moveTurn(30);
  else
	  moveTurn(-30);
	
	delay(1000 * fabs(value) / rotSpeed);
	
	stopWheels();
	adjustParallel(utModule);
}

void Mecanum::moveRight(UTModule& utModule, char sensor, float value){

	float positionThreshold = 1.0;
	float currentPosition, positionTarget;
	float positionError;
	float speed = 30;
	int adj50 = 0;
	
	stopWheels();
	adjustParallel(utModule);
	delay(500);

	if(sensor == 'r'){
		currentPosition = utModule.ultrasonicR();
		positionTarget = currentPosition - value;
		positionError = utModule.ultrasonicR() - positionTarget;
		while(fabs(positionError) > positionThreshold){
			if(positionError>0)
				moveRight(speed);
			else
				moveRight(-speed);
			positionError = utModule.ultrasonicR() - positionTarget;
      if(fabs(positionError-50)<1.0 && adj50==0){
        adjustParallel(utModule);
        adj50 = 1;
      }
		}
	}
	else if(sensor == 'l'){
		currentPosition = utModule.ultrasonicL();
		positionTarget = currentPosition + value;
		positionError = utModule.ultrasonicL() - positionTarget;
		while(fabs(positionError) > positionThreshold){
			if(positionError>0)
				moveRight(speed);
			else
				moveRight(-speed);
			positionError = utModule.ultrasonicL() - positionTarget;
      if(fabs(positionError-50)<1.0 && adj50==0){
        adjustParallel(utModule);
        adj50 = 1;
      }
		}
	}
	stopWheels();
	adjustParallel(utModule);
	delay(500);
}

void Mecanum::moveFront(UTModule& utModule, float value){
	
	float positionThreshold = 1.0;
	float currentPosition, positionTarget;
	float positionError;
	float speed = 30;

	stopWheels();
	adjustParallel(utModule);

		currentPosition = utModule.ultrasonicF();
		positionTarget = value;
		positionError = utModule.ultrasonicF() - positionTarget;
		while(fabs(positionError) > positionThreshold){
			if(positionError>0)
				moveFront(speed);
			else
				moveFront(-speed);
			positionError = utModule.ultrasonicF() - positionTarget;
		}
	stopWheels();
	adjustParallel(utModule);
	
}

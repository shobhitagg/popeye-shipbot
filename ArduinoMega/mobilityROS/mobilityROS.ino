#include "DCMotor.hpp"
#include "Mecanum.hpp"
#include "UTModule.hpp"

#include <ros.h>
#include <string.h>
#include <std_msgs/Int64.h>

//WHEEL GEOMETRY
float lx = 0;
float ly = 0;
float wheelRad = 0;
float dist = 0;

// STATION X,Y				A-1		B-2		C-3		D-4		E-5		F-6		G-7		H-8
float stationX[9] = {15.0,	122.0,	91.0,	61.0,	30.0,	2.0,	3.0,	3.0,	3.0};
float stationY[9] = {30.0,	3.0,	3.0,	3.0,	3.0,	3.0,	2.0,	30.0,	61.0};

int stationLong[] = {1,2,3,4,5};
int stationShort[] = {6,7,8};

//case 1 : bot at the origin and moves to 1-2-3-4-5
//case 2 : bot at the origin and moves to 6-7-8
//case 3 : bot at 1-2-3-4-5 and moves to 1-2-3-4-5
//case 4 : bot at 6-7-8 and moves to 6-7-8
//case 5 : bot at 1-2-3-4-5 and moves to 6-7-8
//case 6 : bot at 6-7-8 and moves to 1-2-3-4-5

// yellow, white, brown/grey, purple/orange
DCMotor M1(19, 40, 11, 34, 1); //ENCA (W), ENCB (Y), PWM, DIR, orientation 
DCMotor M2(18, 41, 12, 35, -1);
DCMotor M3(3, 26, 9, 32, 1);
DCMotor M4(2, 23, 10, 33, -1);

UTModule utModule(47, 46, 49, 48, 24.13); //trigFR, echoFR, trigFL, echoFL, utlx

Mecanum *mecanum;
std_msgs::Int64 currentPosition;
ros::NodeHandle nh;
ros::Publisher publisherObject("chassisEcho", &currentPosition);

bool contains(int* c, int arrSize, int e);
void setupMecanum();

void mecanum_cb(const std_msgs::Int64& cmd_msg){
  int stationToGo = cmd_msg.data%10;
  int stationCurrent = cmd_msg.data/10;

	if(cmd_msg.data == 0){
		if(utModule.ultrasonicL()>utModule.ultrasonicR())
			setupMecanum();
		else{
			mecanum->moveTurn(utModule,90);
			setupMecanum();
		}
		delay(1000);
	}
	else if(cmd_msg.data<100){
		//case 1 : bot at the origin and moves to 1-2-3-4-5
		if(stationCurrent == 0 && contains(stationLong,5,stationToGo)){
			mecanum->moveRight(utModule, 'r', stationX[0]-stationX[stationToGo]);
			mecanum->moveFront(utModule, stationY[stationToGo]);
		}

		//case 2 : bot at the origin and moves to 6-7-8
		else if(stationCurrent == 0 && contains(stationShort,3,stationToGo)){
			mecanum->moveTurn(utModule, -90); //need to see what is the direction of rotation
			mecanum->alignCorner(utModule, 'l', stationX[0], stationY[0]);
			mecanum->moveRight(utModule, 'l', stationX[stationToGo]-stationX[0]);
			mecanum->moveFront(utModule, stationY[stationToGo]);
		}
		//case 3 : bot at 1-2-3-4-5 and moves to 1-2-3-4-5
		else if(contains(stationLong,5,stationCurrent) && contains(stationLong,5,stationToGo)){
			mecanum->moveFront(utModule, 30);
			mecanum->moveRight(utModule, 'r', stationX[stationCurrent]-stationX[stationToGo]);
			mecanum->moveFront(utModule, stationY[stationToGo]);
		}
		//case 4 : bot at 6-7-8 and moves to 6-7-8
		else if(contains(stationShort,3,stationCurrent) && contains(stationShort,3,stationToGo)){
			mecanum->moveFront(utModule, 30);
			mecanum->moveRight(utModule, 'l', stationX[stationToGo]-stationX[stationCurrent]);
			mecanum->moveFront(utModule, stationY[stationToGo]);
		}
		//case 5 : bot at 1-2-3-4-5 and moves to 6-7-8
		else if(contains(stationLong,5,stationCurrent) && contains(stationShort,3,stationToGo)){
			mecanum->moveFront(utModule,30);
			mecanum->moveTurn(utModule, -90); //need to see what is the direction of rotation
			mecanum->alignCorner(utModule, 'l', stationX[0], stationY[0]);
			mecanum->moveRight(utModule, 'l', stationX[stationToGo]-stationX[0]);
			mecanum->moveFront(utModule, stationY[stationToGo]);
		}
		//case 6 : bot at 6-7-8 and moves to 1-2-3-4-5
		else if(contains(stationShort,3,stationCurrent) && contains(stationLong,5,stationToGo)){
			mecanum->moveFront(utModule,30);
			mecanum->moveTurn(utModule, 90); //need to see what is the direction of rotation
			mecanum->alignCorner(utModule, 'r', stationX[0], stationY[0]);
			mecanum->moveRight(utModule, 'r', stationX[0]-stationX[stationToGo]);
			mecanum->moveFront(utModule, stationY[stationToGo]);
		}
		
		currentPosition.data = stationToGo;
		publisherObject.publish(&currentPosition);
	}
	else if(cmd_msg.data>1000 && cmd_msg.data<10000){
		int moveForward = (int)cmd_msg.data/100;
		int moveRight = (int)cmd_msg.data%100 - 50;
		currentPosition.data = moveForward;
		publisherObject.publish(&currentPosition);
		mecanum->moveFront(utModule, moveForward);
		mecanum->moveRight(utModule, 'r', moveRight);
		
	}
	else if(cmd_msg.data == 99999){
		currentPosition.data = utModule.ultrasonicR();
		publisherObject.publish(&currentPosition);
		currentPosition.data = utModule.ultrasonicL();
		publisherObject.publish(&currentPosition);
		currentPosition.data = utModule.ultrasonicF();
		publisherObject.publish(&currentPosition);
	}
		
}

ros::Subscriber<std_msgs::Int64> sub("chassis", &mecanum_cb);

void setup(){
  utModule.RLinitialize(53, 52, 45, 44); //trigR, echoR, trigL, echoL
  pinMode(utModule.frTrig,OUTPUT); pinMode(utModule.frEcho,INPUT);
  pinMode(utModule.flTrig,OUTPUT); pinMode(utModule.flEcho,INPUT);
  pinMode(utModule.rTrig,OUTPUT); pinMode(utModule.rEcho,INPUT);
  pinMode(utModule.lTrig,OUTPUT); pinMode(utModule.lEcho,INPUT);
  
  mecanum = new Mecanum(M1, M2, M3, M4, lx, ly, wheelRad);
  
  pinMode(M1.encaPin, INPUT); pinMode(M1.encbPin, INPUT); pinMode(M1.pwmPin, OUTPUT); pinMode(M1.dirPin, OUTPUT); 
  pinMode(M2.encaPin, INPUT); pinMode(M2.encbPin, INPUT); pinMode(M2.pwmPin, OUTPUT); pinMode(M2.dirPin, OUTPUT);
  pinMode(M3.encaPin, INPUT); pinMode(M3.encbPin, INPUT); pinMode(M3.pwmPin, OUTPUT); pinMode(M3.dirPin, OUTPUT);
  pinMode(M4.encaPin, INPUT); pinMode(M4.encbPin, INPUT); pinMode(M4.pwmPin, OUTPUT); pinMode(M4.dirPin, OUTPUT); 

  Serial.begin(9600);
  Serial.println("initialized");
  nh.initNode();
  nh.subscribe(sub);   
  nh.advertise(publisherObject);
}

void loop(){ 
  nh.spinOnce();
  delay(1);
	mecanum->moveTurn(30);
}

void setupMecanum(){

  for(int i=0; i<2; i++)
    mecanum->alignCorner(utModule, 'r', stationX[0], stationY[0]); // utModule, right, front

  mecanum->adjustParallel(utModule);
}

bool contains(int* c, int arrSize, int e) { 
    for(int i=0; i<arrSize; i++){
        if(c[i] == e)
            return(1);
    }
    
    return(0);
}

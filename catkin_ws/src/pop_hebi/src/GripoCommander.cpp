#include<GripoCommander.h>

void GripoCommander::gripoCB(const pop_hebi::gripoTarget::ConstPtr& receivedMsg){

	//assuming the input from command center is w.r.t. to the arm axes. 
	geometry_msgs::Vector3 xyzTargets;
	
	xyzTargets.x = receivedMsg->x;
	xyzTargets.y = receivedMsg->y;
	xyzTargets.z = receivedMsg->z;
	
	xyzHebi[0] = receivedMsg->x;
	xyzHebi[1] = receivedMsg->y;
	xyzHebi[2] = receivedMsg->z;
	
	valveChange = 0; //need to program this
			
	manipulationStep = 1;
	valveId = receivedMsg->valve;
	
	hebiPub.publish(xyzTargets);

}


void GripoCommander::hebiCB(const std_msgs::UInt32::ConstPtr& recievedMsg){
	
	if (manipulationStep == 1){
		std_msgs::UInt32 value;
		value.data = xyzHebi[2];
		manipulationStep += 1;
		stp1Pub.publish(value);	
	}
	else if(manipulationStep == 4){//send data to manipulate valve already in contact. use stepper or hebi to do so
		switch(valveId){
			case 10:
				break;
				
			case 20:
				break;
				
			case 30:
				break;
				
			case 41:
				break;
			
			case 42:
				break;
				
			case 43:
				break;
				
			case 51:
				break;
				
			case 52:
				break;
			
			default:
				break;	
		}			
		manipulationStep += 1;
	}
	else{
		ROS_INFO_STREAM("Task completed.");
		manipulationStep = 0;
		std_msgs::UInt32 msg;
		msg.data = 1;
		commanderPub.publish(msg);
	}
}

void GripoCommander::stp1CB(const std_msgs::UInt32::ConstPtr& recievedMsg){
	std_msgs::UInt32 value;
	if(manipulationStep == 2){//send data to open/close servo to approach valve
		switch(valveId){
			case 10:
				break;
				
			case 20:
				break;
				
			case 30:
				break;
				
			case 41:
				break;
			
			case 42:
				break;
				
			case 43:
				break;
				
			case 51:
				break;
				
			case 52:
				break;
			
			default:
				break;	
		}
		servoPub.publish(value);
		manipulationStep += 1;
	}
}

void GripoCommander::gripperCB(const std_msgs::UInt32::ConstPtr& recievedMsg){

	if(manipulationStep == 3){
		switch(valveId){
			case 10:
				break;
				
			case 20:
				break;
				
			case 30:
				break;
				
			case 41:
				break;
			
			case 42:
				break;
				
			case 43:
				break;
				
			case 51:
				break;
				
			case 52:
				break;
			
			default:
				break;	
		}
		geometry_msgs::Vector3 xyzTargets;
		xyzTargets.x = xyzHebi[0];
		xyzTargets.y = xyzHebi[1];
		xyzTargets.z = xyzHebi[2];
		hebiPub.publish(xyzTargets);
	}
	
	else{
		ROS_INFO_STREAM("Task completed.");
		manipulationStep = 0;
		std_msgs::UInt32 msg;
		msg.data = 1;
		commanderPub.publish(msg);
	}

}

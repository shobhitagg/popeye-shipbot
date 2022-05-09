#include<GripoCommander.h>

GripoCommander::GripoCommander(ros::NodeHandle& gripo)
{
	ROS_INFO_STREAM("Setting up GriPo node");
	commanderPub = gripo.advertise<std_msgs::Int64>("gripoEcho",1);
	commanderSub = gripo.subscribe<popeye_commander::gripoTarget>("gripo",1,&GripoCommander::gripoCB, this);

	hebiPub = gripo.advertise<popeye_commander::hebiCommand>("hebiTargets",1);                hebiSub = gripo.subscribe<std_msgs::Int64>("hebiTargetsEcho",1,&GripoCommander::hebiCB, this);

	stp1Pub = gripo.advertise<std_msgs::Int64>("stp1Targets",1);
	stp1Sub = gripo.subscribe<std_msgs::Int64>("stp1TargetsEcho",1,&GripoCommander::stp1CB, this);

	stp2Pub = gripo.advertise<std_msgs::Int64>("stp2Targets",1);
	servoPub = gripo.advertise<std_msgs::Int64>("servoTargets",1);

	gripperSub = gripo.subscribe<std_msgs::Int64>("gripperEcho",1,&GripoCommander::gripperCB, this);

	manipulationStep = 0;
	valveId = 0;
	valveChange = 0;

	ROS_INFO_STREAM("GriPo node all set");
}


void GripoCommander::gripoCB(const popeye_commander::gripoTarget::ConstPtr& receivedMsg){

	ROS_INFO_STREAM("Task received");
	//assuming the input from command center is w.r.t. to the arm axes. 
	popeye_commander::hebiCommand msg;
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

	msg.xyz = xyzTargets;
	msg.valve = valveId;	
	ROS_INFO_STREAM("Sending msg to HEBI to position");
	hebiPub.publish(msg);

}


void GripoCommander::hebiCB(const std_msgs::Int64::ConstPtr& recievedMsg){

	ROS_INFO_STREAM("I HEARD BACK FROM HEBI");	
	if (manipulationStep == 1){
		std_msgs::Int64 value;
		value.data = xyzHebi[2];
		manipulationStep = 4;
		hebiPub.publish(value);
		ROS_INFO_STREAM("Sending msg to stepper to adjust");	
	}
	else if(manipulationStep == 4){//send data to manipulate valve already in contact. use stepper or hebi to do so
		int step=0; //step = 1 for hebi, step = 2 for stp
		int value = 0; //defining variable for stp value
		switch(valveId){
			case 10:
				xyzHebi[0] += 0.1;
				step = 1;
				break;
				
			case 20:
				xyzHebi[0] += 0.1;
				step = 1;
				break;
				
			case 30:
				xyzHebi[0] += 0.1;
				step = 1;
				break;
				
			case 41:
				xyzHebi[0] += 0.1;
				step = 1;
				break;
			
			case 42:
				xyzHebi[0] += 0.1;
				step = 1;
				break;
				
			case 43:
				xyzHebi[0] += 0.1;
				step = 1;
				break;
				
			case 51:
				xyzHebi[0] += 0.1;
				step = 1;
				break;
				
			case 52:
				xyzHebi[0] += 0.1;
				step = 1;
				break;
			
			default:
				break;	
		}			
		manipulationStep += 1;
		if(step == 1)//hebi step
		{
			popeye_commander::hebiCommand msg;
			geometry_msgs::Vector3 xyzTargets;
			xyzTargets.x = xyzHebi[0];
			xyzTargets.y = xyzHebi[1];
			xyzTargets.z = xyzHebi[2];
			msg.xyz = xyzTargets;
			msg.valve = valveId;
			hebiPub.publish(msg);

		}
		else if(step ==2)//stepper step
		{
			std_msgs::Int64 msg;
			msg.data = value;
			stp2Pub.publish(msg);
		}
		ROS_INFO_STREAM("Manipulating the valve");
	}
	else{
		ROS_INFO_STREAM("Task completed.");
		manipulationStep = 0;
		std_msgs::Int64 msg;
		msg.data = 1;
		commanderPub.publish(msg);
	}
}

void GripoCommander::stp1CB(const std_msgs::Int64::ConstPtr& recievedMsg){
	ROS_INFO_STREAM("I HEARD BACK FROM STP1");	
	int value;
	if(manipulationStep == 2){//send data to open/close servo to approach valve
		switch(valveId){
			case 10: 
				value = 1;
				break;
				
			case 20:
				value = 1;
				break;
				
			case 30:
				value = 1;
				break;
				
			case 41:
				value = 1;
				break;
			
			case 42:
				value = 1;
				break;
				
			case 43:
				value = 1;
				break;
				
			case 51:
				value = 1;
				break;
				
			case 52:
				value = 1;
				break;
			
			default:
				value = 1;
				break;	
		}
		std_msgs::Int64 msg;
		msg.data = value;
		servoPub.publish(msg);
		manipulationStep += 1;
		ROS_INFO_STREAM("Get the gripper ready");
	}
}

void GripoCommander::gripperCB(const std_msgs::Int64::ConstPtr& recievedMsg){

	ROS_INFO_STREAM("I HEARD BACK FROM gripper");	
	if(manipulationStep == 3){
		switch(valveId){
			case 10:
				xyzHebi[1] += 0.02;
				break;
				
			case 20:
				xyzHebi[1] += 0.02;
				break;
				
			case 30:
				xyzHebi[1] += 0.02;
				break;
				
			case 41:
				xyzHebi[1] += 0.02;
				break;
			
			case 42:
				xyzHebi[1] += 0.02;
				break;
				
			case 43:
				xyzHebi[1] += 0.02;
				break;
				
			case 51:
				xyzHebi[1] += 0.02;
				break;
				
			case 52:
				xyzHebi[1] += 0.02;
				break;
			
			default:
				break;	
		}
		manipulationStep += 1;	
		popeye_commander::hebiCommand msg;
		geometry_msgs::Vector3 xyzTargets;
		xyzTargets.x = xyzHebi[0];
		xyzTargets.y = xyzHebi[1];
		xyzTargets.z = xyzHebi[2];
		msg.xyz = xyzTargets;
		msg.valve = valveId;
		hebiPub.publish(msg);
		ROS_INFO_STREAM("Gripper Ready");
	}
	
	else{
		ROS_INFO_STREAM("Task completed.");
		manipulationStep = 0;
		std_msgs::Int64 msg;
		msg.data = 1;
		commanderPub.publish(msg);
	}

}

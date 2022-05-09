#include<Commander.h>

Commander::Commander(
		ros::NodeHandle& chassis, 
		ros::NodeHandle& gripo, 
		ros::NodeHandle& vision, 
		std::string *goalInput, 
		char *goalStationInput, 
		int missionLengthInput){

		missionLength = missionLengthInput;
		goal = goalInput;
		goalStation = goalStationInput;
		chassisPublisher = chassis.advertise<std_msgs::Int64>("chassis",1);
		chassisSubscriber = chassis.subscribe<std_msgs::Int64>("chassisEcho",1,&Commander::chassisCB, this);
		
		gripoPublisher = gripo.advertise<popeye_commander::gripoTarget>("gripo",1);
		gripoSubscriber = gripo.subscribe<std_msgs::Int64>("gripoEcho",1,&Commander::gripoCB, this);

		visionPublisher = vision.advertise<std_msgs::Int64>("vision",1);
		visionSubscriber = vision.subscribe<popeye_commander::visionRequest>("visionEcho",1,&Commander::visionCB, this);
		
		getMissionParam();
		currentGoalIndex = 0;
		
		std_msgs::Int64 msg;
		msg.data = 0;
		chassisPublisher.publish(msg);
		
		//ROS_INFO_STREAM("All Set. Sending first station");
		msg.data = ((int)goalStation[currentGoalIndex]-64);
		ROS_INFO_STREAM(msg);
		chassisPublisher.publish(msg);	

	}
	

void Commander::chassisCB(const std_msgs::Int64::ConstPtr& receivedMsg){
	ROS_INFO_STREAM("Robot moved to the station");
	ROS_INFO_STREAM(receivedMsg->data);
	ROS_INFO_STREAM("Requesting valve/breaker info from Vision...");
	
	std_msgs::Int64 msg;
	msg.data = 100 + valveId[currentGoalIndex];
	visionPublisher.publish(msg);
}

void Commander::gripoCB(const std_msgs::Int64::ConstPtr& receivedMsg){
	
	ROS_INFO_STREAM("Valve manipulated.");
	
	ROS_INFO_STREAM(receivedMsg->data);
	if(currentGoalIndex < missionLength-1){
		ROS_INFO_STREAM("Moving to the next station...");
		
		ROS_INFO_STREAM(currentGoalIndex);
		std_msgs::Int64 echo_msg;
		echo_msg.data = ((int)goalStation[currentGoalIndex]-64)*10;
		echo_msg.data = echo_msg.data + (int)goalStation[currentGoalIndex+1] - 64;
		ROS_INFO_STREAM(echo_msg.data);
		chassisPublisher.publish(echo_msg);	
		
		currentGoalIndex++;
	}
	else{
		ROS_INFO_STREAM("~~MISSION ACCOMPLISHED~~");
		currentGoalIndex = 0;
	}
}

void Commander::visionCB(const popeye_commander::visionRequest::ConstPtr& receivedMsg){
	
	ROS_INFO_STREAM("Recieved data from the Vision");
	ROS_INFO_STREAM("Forwarding it to the GriPo...");	
	
	popeye_commander::gripoTarget msg;

	msg.x = -0.05;
	msg.y = 0.2;
	msg.z = 0.0851;
	msg.valve = valveId[currentGoalIndex];
	msg.initPos = 0;
	msg.goalPos = 0;
	ROS_INFO_STREAM("Sending this to HEBI");
	ROS_INFO_STREAM(msg);
	gripoPublisher.publish(msg);

}

void Commander::getMissionParam(){

	std::string currentStationValve;
	std::string* goalTemp = goal;
	
	for(int i=0; i<missionLength; i++){
		currentStationValve = goalTemp[i];
		
		if(currentStationValve[0]=='A' || currentStationValve[0]=='B'){
			std::string temp = currentStationValve;
			currentStationValve = "";
			for(int i=0;i<2;i++){
				size_t pos= goalTemp[i].find(" ");
				currentStationValve += goalTemp[i].substr(0,pos);
				goalTemp[i].erase(0, pos + 1);
			}
		}
		else {
			size_t pos= goalTemp[i].find(" ");
			currentStationValve = goalTemp[i].substr(0,pos);	
		}	
		if(currentStationValve.compare("V1") == 0)
			valveId[i] = 10;
		else if (currentStationValve.compare("V2")==0)
			valveId[i] = 20;
		else if (currentStationValve.compare("V3")==0)
			valveId[i] =  30;
		else if (currentStationValve.compare("AB1")==0)
			valveId[i] =  41;
		else if (currentStationValve.compare("AB2")==0)
			valveId[i] =  42;
		else if (currentStationValve.compare("AB3")==0)
			valveId[i] =  43;
		else if (currentStationValve.compare("BB1")==0)
			valveId[i] =  51;
		else if (currentStationValve.compare("BB2")==0)
			valveId[i] = 52;
		else
			valveId[i] = 0;
		
		valveGoalValue[i] = goalTemp[i];		
	}
}

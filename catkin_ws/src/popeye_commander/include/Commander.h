#ifndef __COMMANDER_H_
#define __COMMANDER_H_

#include<ros/ros.h>
#include<string>
#include<std_msgs/String.h>
#include<std_msgs/Int64.h>
#include"popeye_commander/gripoTarget.h"
#include"popeye_commander/visionRequest.h"

class Commander
{
public:
	Commander() {}
	Commander(ros::NodeHandle& chassis, ros::NodeHandle& gripo, ros::NodeHandle& vision, std::string *goalInput, char *goalStationInput, int missionLengthInput);	
	void chassisCB(const std_msgs::Int64::ConstPtr& recievedMsg);
	void gripoCB(const std_msgs::Int64::ConstPtr& recievedMsg);
	void visionCB(const popeye_commander::visionRequest::ConstPtr& recievedMsg);
	
	void getMissionParam();
	
	int missionLength;
	std::string* goal;
	char* goalStation;
	
	int valveId[10];
	int currentGoalIndex;
	std::string valveGoalValue[10];
	
	
	ros::NodeHandle chassis;
	ros::Subscriber chassisSubscriber;
	ros::Publisher chassisPublisher;

	ros::NodeHandle gripo;
	ros::Subscriber gripoSubscriber;
	ros::Publisher gripoPublisher;

	ros::NodeHandle vision;
	ros::Subscriber visionSubscriber;
	ros::Publisher visionPublisher;

};

#endif


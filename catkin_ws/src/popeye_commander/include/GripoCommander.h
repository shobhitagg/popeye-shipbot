#ifndef __GRIPO_COMMANDER_H_
#define __GRIPO_COMMANDER_H_

#include<ros/ros.h>
#include<string>
#include<std_msgs/String.h>
#include<std_msgs/Int64.h>
#include<geometry_msgs/Vector3.h>
#include<popeye_commander/gripoTarget.h>
#include<popeye_commander/hebiCommand.h>

class GripoCommander
{
public:
	GripoCommander(ros::NodeHandle& gripo);

	void gripoCB(const popeye_commander::gripoTarget::ConstPtr& recievedMsg);
	void hebiCB(const std_msgs::Int64::ConstPtr& recievedMsg);
	void stp1CB(const std_msgs::Int64::ConstPtr& recievedMsg);
	void gripperCB(const std_msgs::Int64::ConstPtr& recievedMsg);
	
	ros::Subscriber commanderSub;
	ros::Publisher commanderPub;

	ros::Subscriber hebiSub;
	ros::Publisher hebiPub;

	ros::Subscriber stp1Sub;
	ros::Publisher stp1Pub;
	
	ros::Publisher stp2Pub;
	ros::Publisher servoPub;
	
	ros::Subscriber gripperSub;
	
	int valveId;
	int valveChange;
	int manipulationStep;
	double xyzHebi[3];

};

#endif


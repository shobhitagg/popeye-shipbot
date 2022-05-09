#ifndef __GRIPO_COMMANDER_H_
#define __GRIPO_COMMANDER_H_

#include<ros/ros.h>
#include<string>
#include<std_msgs/String.h>
#include<std_msgs/UInt32.h>
#include<geometry_msgs/Vector3.h>
#include<pop_hebi/gripoTarget.h>

class GripoCommander
{
public:
	GripoCommander(ros::NodeHandle& gripo)
	{
		ROS_INFO_STREAM("Setting up GriPo node");
		commanderPub = gripo.advertise<std_msgs::UInt32>("gripoEcho",1);
		commanderSub = gripo.subscribe<pop_hebi::gripoTarget>("gripo",1,&GripoCommander::gripoCB, this);
		
		hebiPub = gripo.advertise<geometry_msgs::Vector3>("hebiTargets",1);
		hebiSub = gripo.subscribe<std_msgs::UInt32>("hebiTargetsEcho",1,&GripoCommander::hebiCB, this);
		
		stp1Pub = gripo.advertise<std_msgs::UInt32>("stp1Targets",1);
		stp1Sub = gripo.subscribe<std_msgs::UInt32>("stp1TargetsEcho",1,&GripoCommander::stp1CB, this);		
		
		stp2Pub = gripo.advertise<std_msgs::UInt32>("stp2Targets",1);
		servoPub = gripo.advertise<std_msgs::UInt32>("servoTargets",1);
		
		gripperSub = gripo.subscribe<std_msgs::UInt32>("gripperEcho",1,&GripoCommander::gripperCB, this);
		
		manipulationStep = 0;
		valveId = 0;
		valveChange = 0;
		
		ROS_INFO_STREAM("GriPo node all set");
	}
	
	void gripoCB(const pop_hebi::gripoTarget::ConstPtr& recievedMsg);
	void hebiCB(const std_msgs::UInt32::ConstPtr& recievedMsg);
	void stp1CB(const std_msgs::UInt32::ConstPtr& recievedMsg);
	void gripperCB(const std_msgs::UInt32::ConstPtr& recievedMsg);
	
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


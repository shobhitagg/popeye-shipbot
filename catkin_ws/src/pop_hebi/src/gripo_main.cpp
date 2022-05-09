#include <iostream>
#include <fstream>
#include <string>
#include <GripoCommander.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
//include gripotarget.h message header

int main(int argc, char **argv){
	
	ros::init(argc, argv, "gripoCommander");
	
	ROS_INFO_STREAM("INITIALIZING GRIPO...");
	
	//SETUP EACH SUBSYSTEM
	
	ros::NodeHandle nh;
	GripoCommander gripoCommander(nh);
		
	ros::spin();
	
return(0);
}

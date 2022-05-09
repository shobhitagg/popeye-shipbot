#include <iostream>
#include <fstream>
#include <string>
#include <GripoCommander.h>
#include <ros/ros.h>
//include gripotarget.h message header

int main(int argc, char **argv){
	
	ros::init(argc, argv, "gripoCommander");
	
	ROS_INFO_STREAM("INITIALIZING GRIPO...");
	
	//SETUP EACH SUBSYSTEM
	
	ros::NodeHandle nh;
	GripoCommander gripoCommander(nh);
	
	double rate = 50;
	ros::Rate r(rate);

	while(ros::ok()){	
		r.sleep();
		ros::spinOnce();
	}
	
return(0);
}

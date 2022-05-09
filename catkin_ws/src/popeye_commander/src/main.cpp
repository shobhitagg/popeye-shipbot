#include <iostream>
#include <fstream>
#include <string>
#include <Commander.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int64.h>

std::string readMission(std::string fileLocation, char *goalStation, std::string *goal, int &missionLength);
void sortStations(char *goalStation, std::string *goal, int len);

int main(int argc, char **argv){
	
	//READ MISSION FILE
	
	ros::init(argc, argv, "Commander");
	
	char goalStation[10];
	std::string goal[10];
	int missionLength;
	std::string missionCommand = readMission("/home/popeye/catkin_ws/src/popeye_commander/missions/mission.txt", goalStation, goal, missionLength);
	sortStations(goalStation, goal, missionLength);
	
	ROS_INFO_STREAM("MISSION READY. INITIALIZING POPEYE...");
	
	//SETUP EACH SUBSYSTEM
	
	ros::NodeHandle chassis;
	ros::NodeHandle vision;
	ros::NodeHandle gripo;
	
	Commander commander(chassis, gripo, vision, goal, goalStation, missionLength);
		
	ros::spin();
	
return(0);
}

std::string readMission(std::string fileLocation, char *goalStation, std::string *goal, int &missionLength){

	std::fstream missionFile;
	std::string delimiter = ", ";
	size_t pos=0;
	int i=0;
	
	missionFile.open(fileLocation, std::fstream::in);
	
	std::string missionCommand;
	
	if(missionFile.is_open())
		getline(missionFile, missionCommand);
	
	missionFile.close();
		
	ROS_INFO_STREAM(missionCommand);
	while ((pos = missionCommand.find(delimiter)) != std::string::npos) {
		goal[i] = missionCommand.substr(1, pos);
		goalStation[i] = missionCommand[0];
		missionCommand.erase(0, pos + delimiter.length());
		i++;
	}
	
	missionLength = i;	
	
	return(missionCommand);
	
}

void sortStations(char *goalStation, std::string *goal, int len)
{
	int i, j;
	char key;
	std::string keyString;
	
    for (i = 1; i < len; i++)
    {
        key = goalStation[i];
		keyString = goal[i];
        j = i - 1;
 
        /* Move elements of arr[0..i-1], that are
        greater than key, to one position ahead
        of their current position */
        while (j >= 0 && goalStation[j] > key)
        {
            goalStation[j + 1] = goalStation[j];
			goal[j+1] = goal[j];
            j = j - 1;
        }
        goalStation[j + 1] = key;
		goal[j+1] = keyString;
    }

}

/**
 * Put everything together to control a 3-DoF arm.
 *
 * For more information, go to http://docs.hebi.us/tools.html#cpp-api
 *
 * This script assumes you can create a group with 1 module.
 *
 * HEBI Robotics
 * October 2018
 */

#include <iostream>
#include <chrono>
#include <thread>

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/String.h"

#include "hebi_cpp_api/lookup.hpp"
#include "hebi_cpp_api/group_command.hpp"
#include "hebi_cpp_api/group_feedback.hpp"
#include "hebi_cpp_api/robot_model.hpp"
#include "hebi_cpp_api/trajectory.hpp"
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <eigen_conversions/eigen_msg.h>
// The examples provide this utility function for computing gravity compensation
// efforts.
#include "grav_comp.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "hebi_cpp_api/log_file.hpp"

using namespace hebi;
using namespace tf;

/// A helper function to create a group from named modules, and set specified
/// gains on the modules in that group.
std::shared_ptr<Group> getGroup() {
  // Get group
  std::vector<std::string> families {"popeye"};
  std::vector<std::string> names {"0-shoulder","1-elbow","2-wrist"};
  Lookup lookup;
  std::shared_ptr<Group> group = lookup.getGroupFromNames(families, names);
  if (!group)
    return nullptr;
  return group;
}

/// A helper function to actually execute the trajectory on a group of modules

void chatterCallbackCommand()
{

  ROS_INFO_STREAM("I made to checkpoint 1.\n");

    std::shared_ptr<Group> group = getGroup();
  if (!group) {
    std::cout
      << "Group not found, or could not send gains to the modules. Check that the" << std::endl
      << "correct modules on the network, the connection is robust, and that the" << std::endl
      << "gains XML file is in the correct relative path." << std::endl;
  }
  // Load robot model/kinematics and gains
  auto model = robot_model::RobotModel::loadHRDF("/home/popeye/popeyeROS/hrdf/popeye3DOFarm.hrdf");
  if (!model)
  {
    ROS_INFO_STREAM("Could not load HRDF!" << std::endl);
  }
  
  ROS_INFO_STREAM("I made to checkpoint 2.\n");

  GroupFeedback feedback(group->size());
  group->getNextFeedback(feedback);

  Eigen::Matrix4d transform;
  model->getEndEffector(feedback.getPosition(), transform);
  ROS_INFO_STREAM("\nPosition: ");
  ROS_INFO_STREAM(transform.col(3));
  ROS_INFO_STREAM("\nWaypoint: ");
  ROS_INFO_STREAM(feedback.getPosition());


}


int main(int argc, char **argv) 
{
  // Get group of modules and set gains.

  ros::init(argc, argv, "hebi3dof");
  ros::NodeHandle n;

 while(ros::ok()){
  	chatterCallbackCommand();
	std::this_thread::sleep_for(std::chrono::seconds(5));
	ros::spinOnce();
 }
  return 0;
}

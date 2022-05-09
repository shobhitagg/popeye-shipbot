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

    // Set gains
   GroupCommand gains_command(group->size());
   if (!gains_command.readGains("/home/popeye/catkin_ws/src/popeye_commander/src/gains/gains3dof.xml"))
     return nullptr;
   if (!group->sendCommandWithAcknowledgement(gains_command))
     return nullptr;
 
	group->setCommandLifetimeMs(2000);

  return group;
}

/// A helper function to actually execute the trajectory on a group of modules
void executeTrajectory(
		Group& group,
		const robot_model::RobotModel& model,
		const trajectory::Trajectory& trajectory,
		GroupFeedback& feedback) {

  // Set up command object, timing variables, and other necessary variables
  size_t num_joints = group.size();
  GroupCommand command(num_joints);
  double duration = trajectory.getDuration();
  double t = 0;
  Eigen::VectorXd pos_cmd(num_joints);
  Eigen::VectorXd vel_cmd(num_joints);
  Eigen::VectorXd acc_cmd(num_joints); // note that the acceleration command is read from
                                       // the trajectory; you need dynamics info before
                                       // converting to efforts to send to the robot
  Eigen::VectorXd eff_cmd(num_joints);
  //auto start = std::chrono::system_clock::now();
  Eigen::VectorXd masses;
  model.getMasses(masses);

  // GroupFeedback fbk(group->size());
  // group->getNextFeedback(fbk);
  // Eigen::VectorXd currPos(num_joints) = feedback.getPosition();

  while (t < duration) {
	duration = trajectory.getDuration();
    // Get feedback and update the timer
    group.getNextFeedback(feedback);
	
    // Get new commands from the trajectory
    trajectory.getState(t, &pos_cmd, &vel_cmd, &acc_cmd);

    eff_cmd = hebi::util::getGravityCompensationEfforts(model, masses, feedback);

    // Fill in the command and send commands to the arm
    command.setPosition(pos_cmd);
    //command.setVelocity(vel_cmd);
    //command.setEffort(eff_cmd+acc_cmd);
    group.sendCommand(command);
	t += 0.005;
  }
}

void executeTrajectoryArmThenWrist(
    Group& group,
    const robot_model::RobotModel& model,
    const trajectory::Trajectory& trajectory,
    GroupFeedback& feedback) {

  std::vector<std::string> families {"popeye"};
  std::vector<std::string> names {"0-shoulder","1-elbow"};
  Lookup lookup;
  std::shared_ptr<Group> arm_group = lookup.getGroupFromNames(families, names);

  // Set up command object, timing variables, and other necessary variables
  size_t num_joints = group.size();
  GroupCommand command(num_joints);
  GroupCommand arm_command(num_joints-1);
  double duration = trajectory.getDuration();
  double t = 0;
  Eigen::VectorXd pos_cmd(num_joints);
  Eigen::VectorXd vel_cmd(num_joints);
  Eigen::VectorXd acc_cmd(num_joints); // note that the acceleration command is read from
                                       // the trajectory; you need dynamics info before
                                       // converting to efforts to send to the robot
  Eigen::VectorXd eff_cmd(num_joints);
  //auto start = std::chrono::system_clock::now();
  Eigen::VectorXd masses;
  model.getMasses(masses);

  // GroupFeedback fbk(group->size());
  // group->getNextFeedback(fbk);
  // Eigen::VectorXd currPos(num_joints) = feedback.getPosition();

  while (t < duration) {
	duration = trajectory.getDuration();
    // Get feedback and update the timer
    group.getNextFeedback(feedback);
	
    // Get new commands from the trajectory
    trajectory.getState(t, &pos_cmd, &vel_cmd, &acc_cmd);

    eff_cmd = hebi::util::getGravityCompensationEfforts(model, masses, feedback);

    // Fill in the command and send commands to the arm
    arm_command.setPosition(pos_cmd.head(2));
    //command.setVelocity(vel_cmd);
    //command.setEffort(eff_cmd+acc_cmd);
    arm_group->sendCommand(arm_command);
	t += 0.005;
  }
  
	Eigen::VectorXd pos_ar(2);
        pos_ar << pos_cmd.head(2);
	//Eigen::VectorXd vel_ar(2) = vel_cmd.head(2);
	//Eigen::VectorXd acc_ar(2) = acc_cmd.head(2);

  t=0;
  while (t < duration) {
	duration = trajectory.getDuration();
    // Get feedback and update the timer
    group.getNextFeedback(feedback);
	
    // Get new commands from the trajectory
    trajectory.getState(t, &pos_cmd, &vel_cmd, &acc_cmd);

    eff_cmd = hebi::util::getGravityCompensationEfforts(model, masses, feedback);

	pos_cmd.head(2) = pos_ar.head(2);
	//vel_cmd.head(2) = acc_ar.head(2);
	//acc_cmd.head(2) = acc_ar.head(2);

    // Fill in the command and send commands to the arm
    command.setPosition(pos_cmd);
    //command.setVelocity(vel_cmd);
    //command.setEffort(eff_cmd+acc_cmd);
    group.sendCommand(command);
	t += 0.005;
  }

}


void chatterCallbackCommand(const geometry_msgs::Vector3 &xyzTargets)
{
  // Convert these to joint angle waypoints using IK solutions for each of the
  // xyz locations.  Copy the initial waypoint at the end so we close the square
  
  // Choose an "elbow up" initial configuration for IK
  //

  Eigen::Vector3d xyz_targets;
  tf::vectorMsgToEigen(xyzTargets, xyz_targets);

  Eigen::Vector3d elbow_up_angles;
  elbow_up_angles << M_PI, 0, 0;
//  elbow_up_angles << 0, 0, 0;

  Eigen::MatrixXd joint_targets(3, 2);
  Eigen::VectorXd ik_res_angles;

  ROS_INFO_STREAM("I made to checkpoint 1.\n");

  std::shared_ptr<Group> group = getGroup();
  if (!group) {
    std::cout
      << "Group not found, or could not send gains to the modules. Check that the" << std::endl
      << "correct modules on the network, the connection is robust, and that the" << std::endl
      << "gains XML file is in the correct relative path." << std::endl;
  }
  // Load robot model/kinematics and gains
  auto model = robot_model::RobotModel::loadHRDF("/home/popeye/catkin_ws/src/popeye_commander/src/hrdf/popeye3DOFarm.hrdf");
  if (!model)
  {
    ROS_INFO_STREAM("Could not load HRDF!" << std::endl);
  }
  
  model->solveIK(
 		 elbow_up_angles, // Initial joint angles
 		 ik_res_angles, // IK result
 		 robot_model::EndEffectorPositionObjective(xyz_targets)); // Objective

  joint_targets.col(0) = ik_res_angles.col(0);
  joint_targets.col(1) = joint_targets.col(0);

  ROS_INFO_STREAM(joint_targets);

  ROS_INFO_STREAM("I made to checkpoint 2.");

  GroupFeedback feedback(group->size());
  group->getNextFeedback(feedback);
// Get a trajectory from the current posiffffgto the first corner of the box: 
  Eigen::MatrixXd waypoints(3, 2);
  waypoints.col(0) = feedback.getPosition();
  waypoints.col(1) = joint_targets.col(0);//joint_targets.col(0);
  Eigen::VectorXd time(2);
  time << 0, 5; // Seconds for the motion; we do this slowly
  
  auto trajectory = hebi::trajectory::Trajectory::createUnconstrainedQp(time, waypoints);

  // Call a helper function (above) to execute this motion on the robot
  executeTrajectoryArmThenWrist(*group, *model, *trajectory, feedback);
  
  ROS_INFO_STREAM("\nTarget : ");
  ROS_INFO_STREAM(joint_targets.col(0));
  
  group->getNextFeedback(feedback);
  ROS_INFO_STREAM("\nActual: ");
  ROS_INFO_STREAM(feedback.getPosition());
}


int main(int argc, char **argv) 
{
  // Get group of modules and set gains.

  ros::init(argc, argv, "hebi3dof");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallbackCommand);
  ros::spin();

  return 0;
}

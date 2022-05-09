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
void executeTrajectory(
    Group& group,
    const robot_model::RobotModel& model,
    const trajectory::Trajectory& trajectory,
    GroupFeedback& feedback) {

  // Set up command object, timing variables, and other necessary variables
  size_t num_joints = group.size();
  GroupCommand command(num_joints);
  double duration = trajectory.getDuration();
  Eigen::VectorXd pos_cmd(num_joints);
  Eigen::VectorXd vel_cmd(num_joints);
  Eigen::VectorXd acc_cmd(num_joints); // note that the acceleration command is read from
                                       // the trajectory; you need dynamics info before
                                       // converting to efforts to send to the robot
  Eigen::VectorXd eff_cmd(num_joints);
  auto start = std::chrono::system_clock::now();
  std::chrono::duration<double> t(std::chrono::system_clock::now() - start);
  Eigen::VectorXd masses;
  model.getMasses(masses);

  while (t.count() < duration) {
    // Get feedback and update the timer
    group.getNextFeedback(feedback);
    t = std::chrono::system_clock::now() - start;

    // Get new commands from the trajectory
    trajectory.getState(t.count(), &pos_cmd, &vel_cmd, &acc_cmd);

    // Calculate commanded efforts to assist with tracking the trajectory.
    // Gravity Compensation uses knowledge of the arm's kinematics and mass to
    // compensate for the weight of the arm.  Dynamic Compensation uses the
    // kinematics and mass to compensate for the commanded accelerations of
    // the arm.
    eff_cmd = hebi::util::getGravityCompensationEfforts(model, masses, feedback);
    // NOTE: dynamic compensation effort computation has not yet been added to
    // the C++ API utility functions.  These are coming soon! TODO
    // eff_cmd += hebi::util::DynamicCompensation::getEfforts(...);

    // Fill in the command and send commands to the arm
    command.setPosition(pos_cmd);
    //command.setVelocity(vel_cmd);
    //command.setEffort(eff_cmd+acc_cmd);
    group.sendCommand(command);
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
  auto model = robot_model::RobotModel::loadHRDF("/home/popeye/popeyeROS/hrdf/popeye3DOFarm.hrdf");
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

  Eigen::Matrix4d transform;
  model->getEndEffector(feedback.getPosition(), transform);
  ROS_INFO_STREAM("\nOriginal: ");
  ROS_INFO_STREAM(transform);
  
  // Call a helper function (above) to execute this motion on the robot
  executeTrajectory(*group, *model, *trajectory, feedback);
  
  model->getEndEffector(feedback.getPosition(), transform);
  ROS_INFO_STREAM("\nTarget : ");
  ROS_INFO_STREAM(transform);
  
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

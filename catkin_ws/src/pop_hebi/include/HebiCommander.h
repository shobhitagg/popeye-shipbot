#ifndef __HEBI_COMMANDER_H_
#define __HEBI_COMMANDER_H_

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

class HebiArm{

	public:
		Eigen::VectorXd currentPoint;
		Eigen::VectorXd currentWaypoint;
		Eigen::VectorXd nextWaypoint;
		Eigen::VectorXd homeWaypoint;
		bool nextPoint;
		Eigen::MatrixXd wristRotation;
		std::shared_ptr<hebi::Group> group;
		std::shared_ptr<hebi::Group> arm_group;
		std::unique_ptr<hebi::robot_model::RobotModel> model;
		
		ros::NodeHandle ikSolver;
		ros::Subscriber ikSolverSub;
		
		HebiArm(void);
		
		void ikSolverCB(const geometry_msgs::Vector3 &xyzTargets);
		void fkSolver();
};

#endif

#include<HebiCommander.h>
#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/String.h"

#define TIMESTEP 0.005

int main(int argc, char **argv) 
{
  // Get group of modules and set gains.

	ros::init(argc, argv, "hebiCommand");
	
	ROS_INFO_STREAM("Starting up HEBI command center");
	HebiArm hebiBoi;

	double rate = 20;
	ros::Rate r(rate);
	
	while(ros::ok()){
		  Eigen::Vector3d elbow_up_angles;
		  elbow_up_angles << M_PI, 0, 0;
		  Eigen::VectorXd time(2);
		  time << 0, 5; // Seconds for the motion; we do this slowly
		  
		  //create trajectory here------
		  hebiBoi.fkSolver();
		  Eigen::MatrixXd waypoints(3, 2);
		  waypoints.col(0) = hebiBoi.currentWaypoint;
		  
		  if(hebiBoi.nextPoint){
				waypoints.col(1) = hebiBoi.nextWaypoint;
		  }
		  else{
				waypoints.col(1) = hebiBoi.currentWaypoint;
				auto trajectory = hebi::trajectory::Trajectory::createUnconstrainedQp(time, waypoints);
		  }
		  auto trajectory = hebi::trajectory::Trajectory::createUnconstrainedQp(time, waypoints);
		  //-----------------
		  // Set up command object, timing variables, and other necessary variables
		  size_t num_joints = hebiBoi.group->size();
		  size_t arm_num_joints = hebiBoi.arm_group->size();
		  hebi::GroupCommand command(num_joints);
		  hebi::GroupCommand arm_command(arm_num_joints);
		  double duration = trajectory->getDuration();
		  double t = 0;
		  Eigen::VectorXd pos_cmd(num_joints);
		  Eigen::VectorXd vel_cmd(num_joints);
		  Eigen::VectorXd acc_cmd(num_joints);
		  Eigen::VectorXd eff_cmd(num_joints);
		  //auto start = std::chrono::system_clock::now();
		  Eigen::VectorXd masses;
		  hebiBoi.model->getMasses(masses);

		  while (t < duration) {
			duration = trajectory->getDuration();
	
			// Get new commands from the trajectory
			trajectory->getState(t, &pos_cmd, &vel_cmd, &acc_cmd);

			//eff_cmd = hebi::util::getGravityCompensationEfforts(hebiBoi.model, masses, hebiBoi.feedback);

			// Fill in the command and send commands to the arm
			arm_command.setPosition(pos_cmd.head(2));
			hebiBoi.arm_group->sendCommand(arm_command);
			t += TIMESTEP;
		  }
		  
		  Eigen::VectorXd pos_ar(2);
		  pos_ar << pos_cmd.head(2);

		  t=0;
		  while (t < duration) {
			duration = trajectory->getDuration();
			
			// Get new commands from the trajectory
			trajectory->getState(t, &pos_cmd, &vel_cmd, &acc_cmd);

			//eff_cmd = hebi::util::getGravityCompensationEfforts(hebiBoi.model, masses, hebiBoi.feedback);

			pos_cmd.head(2) = pos_ar.head(2);

			// Fill in the command and send commands to the arm
			command.setPosition(pos_cmd);
			hebiBoi.group->sendCommand(command);
			t += TIMESTEP;
		  }
		
		hebiBoi.nextPoint = 0;
		
		r.sleep();
		ros::spinOnce();
	}
  return 0;
}

#include<HebiCommander.h>
#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Int64.h"

#define TIMESTEP 0.005

Eigen::VectorXd executeTrajectoryArmThenWrist(
    hebi::Group& group,
    const hebi::trajectory::Trajectory& trajectory,
    hebi::GroupFeedback& feedback);

int main(int argc, char **argv) 
{
  // Get group of modules and set gains.

	ros::init(argc, argv, "hebiCommand");
	
	int ctr = 0;

	ROS_INFO_STREAM("Starting up HEBI command center");
	HebiCommander hebiBoi;

	double rate = 50;
	ros::Rate r(rate);
	
	while(ros::ok()){
		  Eigen::Vector3d elbow_up_angles;
		  elbow_up_angles << M_PI, 0, 0;
		  Eigen::VectorXd time(2);
		  time << 0, 5; // Seconds for the motion; we do this slowly
		  
		  //create trajectory here------
		  hebiBoi.fkSolver();
		  int nextPoint=0;

		  hebi::GroupFeedback feedback(hebiBoi.group->size());
		  hebiBoi.group->getNextFeedback(feedback);

		  if(hebiBoi.nextPoint==1){
			  ROS_INFO_STREAM("Pursuing Next Point");
			  Eigen::MatrixXd waypoints(3, 2);
			  waypoints.col(0) = hebiBoi.currentWaypoint;
  			  waypoints.col(1) = hebiBoi.nextWaypoint;
			  nextPoint = 1;
					
			  auto trajectory = hebi::trajectory::Trajectory::createUnconstrainedQp(time, waypoints);
			  hebiBoi.lastPosCmd = executeTrajectoryArmThenWrist(*hebiBoi.group, *trajectory, feedback);

		  }
		  else{
  		        ROS_INFO_STREAM("Holding Position");
			hebi::GroupCommand command(3);
			command.setPosition(hebiBoi.lastPosCmd);
			hebiBoi.group->sendCommand(command);
		  }  
		  if(nextPoint){
			hebiBoi.nextPoint = 0;
			std_msgs::Int64 msg;
			msg.data = 1;
			if(ctr !=0)
				hebiBoi.Pub.publish(msg);
			ctr++;
		  }
		
		  r.sleep();
		  ros::spinOnce();
	}
  return 0;
}

Eigen::VectorXd executeTrajectoryArmThenWrist(
    hebi::Group& group,
    const hebi::trajectory::Trajectory& trajectory,
    hebi::GroupFeedback& feedback) {

  std::vector<std::string> families {"popeye"};
  std::vector<std::string> names {"0-shoulder","1-elbow"};
  hebi::Lookup lookup;
  std::shared_ptr<hebi::Group> arm_group = lookup.getGroupFromNames(families, names);

  // Set up command object, timing variables, and other necessary variables
  size_t num_joints = group.size();
  hebi::GroupCommand command(num_joints);
  hebi::GroupCommand arm_command(num_joints-1);
  double duration = trajectory.getDuration();
  double t = 0;
  Eigen::VectorXd pos_cmd(num_joints);
  Eigen::VectorXd vel_cmd(num_joints);
  Eigen::VectorXd acc_cmd(num_joints);

  while (t < duration) {
	duration = trajectory.getDuration();
    // Get feedback and update the timer
    group.getNextFeedback(feedback);
	
    // Get new commands from the trajectory
    trajectory.getState(t, &pos_cmd, &vel_cmd, &acc_cmd);

    // Fill in the command and send commands to the arm
    arm_command.setPosition(pos_cmd.head(2));
    arm_group->sendCommand(arm_command);
	t += TIMESTEP;
  }
  
	Eigen::VectorXd pos_ar(2);
        pos_ar << pos_cmd.head(2);

  t=0;
  while (t < duration) {
	duration = trajectory.getDuration();
    // Get feedback and update the timer
    group.getNextFeedback(feedback);
	
    // Get new commands from the trajectory
    trajectory.getState(t, &pos_cmd, &vel_cmd, &acc_cmd);

    pos_cmd.head(2) = pos_ar.head(2);

    // Fill in the command and send commands to the arm
    command.setPosition(pos_cmd);
    group.sendCommand(command);
	t += TIMESTEP;
  }
  return(pos_cmd);

}

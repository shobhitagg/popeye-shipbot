#include<HebiCommander.h>

HebiArm::HebiArm(void)
{
	ROS_INFO_STREAM("Initializing Arm");
	std::vector<std::string> families {"popeye"};
	std::vector<std::string> names {"0-shoulder","1-elbow","2-wrist"};
	hebi::Lookup lookup;
	group = lookup.getGroupFromNames(families, names);
	
	std::vector<std::string> arm_names {"0-shoulder","1-elbow"};
	arm_group = lookup.getGroupFromNames(families, arm_names);	
	
	// Set gains
	hebi::GroupCommand gains_command(group->size());
	gains_command.readGains("/home/popeye/catkin_ws/src/pop_hebi/src/gains/gains3dof.xml");
	group->sendCommandWithAcknowledgement(gains_command);
		
	if (!group) {
	ROS_INFO_STREAM(
	  "Group not found, or could not send gains to the modules. Check that the" << std::endl
	  << "correct modules on the network, the connection is robust, and that the" << std::endl
	  << "gains XML file is in the correct relative path." << std::endl);
	}
	// Load robot model/kinematics and gains
	model = hebi::robot_model::RobotModel::loadHRDF("/home/popeye/catkin_ws/src/pop_hebi/src/hrdf/popeye3DOFarm.hrdf");
	if (!model)
	{
		ROS_INFO_STREAM("Could not load HRDF!" << std::endl);
	}
	
	//ikSolverSub = ikSolver.subscribe<geometry_msgs::Vector3>("solveIK",1,&HebiArm::ikSolverCB, this);
	ikSolverSub = ikSolver.subscribe("hebiTargets",1,&HebiArm::ikSolverCB, this);

	hebi::GroupFeedback feedback(group->size());
	group->getNextFeedback(feedback);
	currentWaypoint << feedback.getPosition();
	
	homeWaypoint << M_PI, 0.75*M_PI, 0;
	nextPoint = 1;
	nextWaypoint << homeWaypoint;		
	ROS_INFO_STREAM("Arm ready to go...");	
}

void HebiArm::ikSolverCB(const geometry_msgs::Vector3 &xyzTargets)
{
	while(nextPoint){}

	Eigen::Vector3d xyz_targets;
	tf::vectorMsgToEigen(xyzTargets, xyz_targets);

	Eigen::Vector3d elbow_up_angles;
	elbow_up_angles << M_PI, 0, 0;
	Eigen::VectorXd ik_res_angles;

	
	model->solveIK(
		elbow_up_angles, // Initial joint angles
		ik_res_angles, // IK result
		hebi::robot_model::EndEffectorPositionObjective(xyz_targets)); // Objective
	
	nextWaypoint = ik_res_angles;
	nextPoint = 1;
}

void HebiArm::fkSolver()
{
	hebi::GroupFeedback feedback(group->size());
	group->getNextFeedback(feedback);		

	Eigen::Matrix4d transform;
	model->getEndEffector(feedback.getPosition(), transform);
	
	currentWaypoint << feedback.getPosition();
	currentPoint << transform.col(3).head(3);
}

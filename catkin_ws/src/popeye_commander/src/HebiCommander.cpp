#include<HebiCommander.h>

HebiCommander::HebiCommander(void)
{
	ROS_INFO_STREAM("Initializing Arm");
	std::vector<std::string> families {"popeye"};
	std::vector<std::string> names {"0-shoulder","1-elbow","2-wrist"};
	std::vector<std::string> arm_names {"0-shoulder","1-elbow"};
	hebi::Lookup lookup;
	group = lookup.getGroupFromNames(families, names);	
	arm_group = lookup.getGroupFromNames(families, arm_names);	
	
	// Set gains
	hebi::GroupCommand gains_command(group->size());
	gains_command.readGains("/home/popeye/catkin_ws/src/popeye_commander/src/gains/gains3dof.xml");
	group->sendCommandWithAcknowledgement(gains_command);
	group->setCommandLifetimeMs(5000);

	if (!group) {
	ROS_INFO_STREAM(
	  "Group not found, or could not send gains to the modules. Check that the" << std::endl
	  << "correct modules on the network, the connection is robust, and that the" << std::endl
	  << "gains XML file is in the correct relative path." << std::endl);
	}
	// Load robot model/kinematics and gains
	model = hebi::robot_model::RobotModel::loadHRDF("/home/popeye/catkin_ws/src/popeye_commander/src/hrdf/popeye3DOFarm.hrdf");
	if (!model)
	{
		ROS_INFO_STREAM("Could not load HRDF!" << std::endl);
	}
	
	//ikSolverSub = ikSolver.subscribe<geometry_msgs::Vector3>("solveIK",1,&HebiArm::ikSolverCB, this);
	ikSolverSub = ikSolver.subscribe("hebiTargets",1,&HebiCommander::ikSolverCB, this);
	Pub = ikSolver.advertise<std_msgs::Int64>("hebiTargetsEcho",1);

	hebi::GroupFeedback feedback(group->size());
	group->getNextFeedback(feedback);
	
	currentWaypoint= feedback.getPosition();
	ROS_INFO_STREAM("arm ready to go");
	homeWaypoint ={ 3.45, 2.80, 1.57};
	nextPoint = 1;
	nextWaypoint = homeWaypoint;		
	ROS_INFO_STREAM("Arm ready to go...");	
}

void HebiCommander::ikSolverCB(const popeye_commander::hebiCommand &receivedMsg)
{
	ROS_INFO_STREAM("callback");
	while(nextPoint){}
	
	geometry_msgs::Vector3 temp = receivedMsg.xyz;

	Eigen::Vector3d xyz_targets;
	Eigen::Matrix3d so3;
	tf::vectorMsgToEigen(temp, xyz_targets);

	switch(receivedMsg.valve){
		case 10:
			so3 <<	1, 0, 0,
				0, 1, 0,
				0, 0, 1;
			break;
		case 20:
			so3 <<	1, 0, 0,
				0, 1, 0,
				0, 0, 1;
			break;
		case 30:
			so3 <<	1, 0, 0,
				0, 1, 0,
				0, 0, 1;
			break;
		case 41:
			so3 <<	1, 0, 0,
				0, 1, 0,
				0, 0, 1;
			break;
		case 42:
			so3 <<	1, 0, 0,
				0, 1, 0,
				0, 0, 1;
			break;

		case 43:
			so3 <<	1, 0, 0,
				0, 1, 0,
				0, 0, 1;
			break;

		case 51:
			so3 <<	1, 0, 0,
				0, 1, 0,
				0, 0, 1;
			break;
		case 52:
			so3 <<	1, 0, 0,
				0, 1, 0,
				0, 0, 1;
			break;

		default:
			so3 <<	1, 0, 0,
				0, 1, 0,
				0, 0, 1;
			break;
	}
	
	Eigen::Vector3d elbow_up_angles;
	elbow_up_angles << M_PI, 0, 0;
	Eigen::VectorXd ik_res_angles;

	Eigen::VectorXd min_positions(model->getDoFCount());
	min_positions << 2.0f, 0.0f, -1.8f;
	Eigen::VectorXd max_positions(model->getDoFCount());
	max_positions << 3.5f, 3.0f, 1.6f;
		
	model->solveIK(
		elbow_up_angles, // Initial joint angles
		ik_res_angles, // IK result
		hebi::robot_model::EndEffectorPositionObjective(xyz_targets),
		hebi::robot_model::EndEffectorSO3Objective(so3),
		hebi::robot_model::JointLimitConstraint(min_positions, max_positions)); // Objective
	ROS_INFO_STREAM("Target angles:");
	ROS_INFO_STREAM(ik_res_angles);	
	nextWaypoint = ik_res_angles;

	if(receivedMsg.valve == 99)
		nextWaypoint = homeWaypoint;

	nextPoint = 1;
}

void HebiCommander::fkSolver()
{
	hebi::GroupFeedback feedback(group->size());
	group->getNextFeedback(feedback);		

	Eigen::Matrix4d transform;
	model->getEndEffector(feedback.getPosition(), transform);
	
	currentWaypoint = feedback.getPosition();
	currentPoint = transform.col(3).head(3);
}

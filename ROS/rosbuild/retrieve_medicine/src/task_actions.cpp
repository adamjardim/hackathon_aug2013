#include <retrieve_medicine/task_actions.h>

using namespace std;

taskActions::taskActions() : 
	acMoveBase("/move_base", true),
	acMoveHead("/head_traj_controller/point_head_action", true),
	acMoveTorso("/torso_controller/position_joint_action", true),
	acLeftArm("/l_arm_controller/joint_trajectory_action", true),
	acRightArm("/r_arm_controller/joint_trajectory_action", true),
	acLeftGripper("/l_gripper_controller/gripper_action", true),
	acRightGripper("/r_gripper_controller/gripper_action", true),
	acSegment("/object_detection_user_command", true),
	acTuckArms("/tuck_arms", true),
	acIMGUI("/imgui_action", true),
	asNavigate(n, "navigate_action", boost::bind(&taskActions::executeNavigate, this, _1), false),
	asHandoff(n, "handoff_action", boost::bind(&taskActions::executeHandoff, this, _1), false),
	asBackup(n, "backup_action", boost::bind(&taskActions::executeBackup, this, _1), false),
	asPickupAll(n, "pickup_all_action", boost::bind(&taskActions::executePickupAll, this, _1), false),
	asRelease(n, "release_action", boost::bind(&taskActions::executeRelease, this, _1), false)
{
	hasSegmented = false;

	ROS_INFO("Waiting for move_base action server...");
	acMoveBase.waitForServer();
	ROS_INFO("Finished waiting for move_base action server.");
	
	ROS_INFO("Waiting for right gripper action server...");
	acRightGripper.waitForServer();
	ROS_INFO("Finsihed waiting for right gripper action server");
	
	ROS_INFO("Waiting for left gripper action server...");
	acLeftGripper.waitForServer();
	ROS_INFO("Finsihed waiting for left gripper action server");
	
	ROS_INFO("Waiting for move head action server...");
	acMoveHead.waitForServer();
	ROS_INFO("Finished waiting for move head action server.");
	
	ROS_INFO("Waiting for right arm action server...");
	acMoveBase.waitForServer();
	ROS_INFO("Finished waiting for right arm action server.");
	
	ROS_INFO("Waiting for left arm action server...");
	acMoveBase.waitForServer();
	ROS_INFO("Finished waiting for left arm action server.");
	
	ROS_INFO("Waiting for torso action server...");
	acMoveTorso.waitForServer();
	ROS_INFO("Finished waiting for torso action server.");

	ROS_INFO("Waiting for IMGUI action server...");
	acIMGUI.waitForServer();
	ROS_INFO("Finished waiting for IMGUI action server.");

	asNavigate.start();
	asHandoff.start();
	asBackup.start();
	asPickupAll.start();
	asRelease.start();

	baseCommandPublisher = n.advertise<geometry_msgs::Twist>("/base_controller/command", -1);

    position_client = n.serviceClient<position_server::GetPosition>("position_server/get_position");
    graspCheckClient = n.serviceClient<interactive_world_hackathon::GraspCheck>("interactive_world_hackathon/grasp_check");
    
    //Define arm joint positions
	string rightJoints[] = {"r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", "r_wrist_roll_joint"};
	string leftJoints[] = {"l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"};
	leftArmJointNames.assign(leftJoints, leftJoints + 7);
	rightArmJointNames.assign(rightJoints, rightJoints + 7);
	double leftSidePos[] = {2.115, 0.0, 1.64, -2.07, 1.64, -1.680, 1.398};
	double rightSidePos[] = {-2.115, 0.0, -1.64, -2.07, -1.64, -1.680, 1.398};
	leftArmSidePosition.assign(leftSidePos, leftSidePos + 7);
	rightArmSidePosition.assign(rightSidePos, rightSidePos + 7);
	double leftHandoffPos[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double rightHandoffPos[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	leftArmHandoffPosition.assign(leftHandoffPos, leftHandoffPos + 7);
	rightArmHandoffPosition.assign(rightHandoffPos, rightHandoffPos + 7);
	
	basePoseSubscriber = n.subscribe("/robot_pose", 1, &taskActions::basePoseCallback, this);
	objectSubscriber = n.subscribe("/interactive_object_recognition_result", 1, &taskActions::objectCallback, this);
}

void taskActions::executeNavigate(const retrieve_medicine::navigateGoalConstPtr& goal)
{
	ROS_INFO("Goal task name: %s", goal->taskName.c_str());
	
	position_server::GetPosition srv;
    srv.request.positionName = goal->taskName;
    if (!position_client.call(srv))
    {
    	ROS_INFO("Invalid task name, action could not finish");
    	asNavigate.setPreempted();
    	return;
    }
	
	geometry_msgs::PoseStamped target;
	target.header.frame_id = "/map";
	target.pose.position.x = srv.response.position.pose.x;
	target.pose.position.y = srv.response.position.pose.y;
	target.pose.position.z = 0.0;
	target.pose.orientation.x = 0.0;
	target.pose.orientation.y = 0.0;
	target.pose.orientation.z = sin(srv.response.position.pose.theta/2.0);
	target.pose.orientation.w = cos(srv.response.position.pose.theta/2.0);
	move_base_msgs::MoveBaseGoal moveGoal;
	
	float dstToGoal = sqrt(pow(basePose.position.x - target.pose.position.x, 2) + pow(basePose.position.y - target.pose.position.y, 2));
	float navSuccessThreshold = .3;
	
	ROS_INFO("Distance to nav goal: %f", dstToGoal);
	
	if (dstToGoal > navSuccessThreshold)
	{
		ROS_INFO("Tucking arms");

		pr2_common_action_msgs::TuckArmsGoal armTuckGoal;
		armTuckGoal.tuck_left = true;
		armTuckGoal.tuck_right = true;
		acTuckArms.sendGoal(armTuckGoal);
		acTuckArms.waitForResult(ros::Duration(15));

		ROS_INFO("Navigating to requested position");

		moveGoal.target_pose = target;
		acMoveBase.sendGoal(moveGoal);

		acMoveBase.waitForResult(ros::Duration(30));
	

		dstToGoal = sqrt(pow(basePose.position.x - target.pose.position.x, 2) + pow(basePose.position.y - target.pose.position.y, 2));
	
		ROS_INFO("Distance to nav goal: %f", dstToGoal);
	

		//The action server for autonomous base navigation has a bug where it 
		//often reports unsuccessful navigation as soon as it finishes,
		//the goal is sent a second time to get around this bug.
		if (dstToGoal > navSuccessThreshold || acMoveBase.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			acMoveBase.sendGoal(moveGoal);
			acMoveBase.waitForResult(ros::Duration(15.0));
			dstToGoal = sqrt(pow(basePose.position.x - target.pose.position.x, 2) + pow(basePose.position.y - target.pose.position.y, 2));
		}
	}

	if (dstToGoal < navSuccessThreshold || acMoveBase.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("Untuck arms and adjust height");
		//Untuck arms and set torso height
		pr2_common_action_msgs::TuckArmsGoal armUntuckGoal;
		armUntuckGoal.tuck_left = false;
		armUntuckGoal.tuck_right = false;
		acTuckArms.sendGoal(armUntuckGoal);		
		acTuckArms.waitForResult(ros::Duration(10));
		
		pr2_controllers_msgs::SingleJointPositionGoal torsoGoal;
		torsoGoal.position = srv.response.position.height;
		if (torsoGoal.position < 0.0)
			torsoGoal.position = 0.0;
		else if (torsoGoal.position > 0.6)
			torsoGoal.position = 0.6;
		acMoveTorso.sendGoal(torsoGoal);
		acMoveTorso.waitForResult(ros::Duration(20));
		
		if (goal->align)
		{
			ros::Rate r(60);
			
			//Move arms to side
			ROS_INFO("Moving arms to side position");
			pr2_controllers_msgs::JointTrajectoryGoal leftArmGoal;
			leftArmGoal.trajectory.joint_names = leftArmJointNames;
			trajectory_msgs::JointTrajectoryPoint leftSidePoint;
			leftSidePoint.positions = leftArmSidePosition;
			leftSidePoint.time_from_start = ros::Duration(3);
			leftArmGoal.trajectory.points.push_back(leftSidePoint);
			pr2_controllers_msgs::JointTrajectoryGoal rightArmGoal;
			rightArmGoal.trajectory.joint_names = rightArmJointNames;
			trajectory_msgs::JointTrajectoryPoint rightSidePoint;
			rightSidePoint.positions = rightArmSidePosition;
			rightSidePoint.time_from_start = ros::Duration(3);
			rightArmGoal.trajectory.points.push_back(rightSidePoint);
			acLeftArm.sendGoal(leftArmGoal);
			acRightArm.sendGoal(rightArmGoal);
			acLeftArm.waitForResult(ros::Duration(6));
			acRightArm.waitForResult(ros::Duration(6));
			
			//Get align to table position
			string alignTaskName = goal->taskName;
			alignTaskName = alignTaskName.substr(0, alignTaskName.length() - 3);
			alignTaskName = alignTaskName.append("Table Align");
			srv.request.positionName = alignTaskName;
			
			if (!position_client.call(srv))
			{
				ROS_INFO("Invalid task name, action could not finish");
				asNavigate.setPreempted();
				return;
			}
			
			//Align to table with lower-level control
			ROS_INFO("Aligning to table");
			vec3 pickupPos;
			pickupPos.x = srv.response.position.pose.x;
			pickupPos.y = srv.response.position.pose.y;
			pickupPos.t = srv.response.position.pose.theta;
			
			float currentHeading = asin(basePose.orientation.z) * 2;
			
			float xError = pickupPos.x - basePose.position.x;
			float yError = pickupPos.y - basePose.position.y;
			float turnError = pickupPos.t - currentHeading;
			if (turnError < -PI)
				turnError += 2*PI;
			else if (turnError > PI)
				turnError -= 2*PI;
			
			float totalTurnError = 0;
			float totalXError = 0;
			float totalYError = 0;
			float prevTurnError = 0;
			float prevXError = xError;
			float prevYError = yError;
			
			while (fabs(turnError) > .0873 || fabs(xError) > .05 || fabs(yError) > .05)
			{
				currentHeading = asin(basePose.orientation.z) * 2;
				
				xError = pickupPos.x - basePose.position.x;
				yError = pickupPos.y - basePose.position.y;
				turnError = pickupPos.t - currentHeading;
				if (turnError < -PI)
					turnError += 2*PI;
				else if (turnError > PI)
					turnError -= 2*PI;
				
				float xVel = 0.0;
				float yVel = 0.0;
				float tVel = 0.0;
				
				//correct orientation
				if (fabs(turnError) > .05)
				{
					totalXError = 0;
					totalYError = 0;
					tVel = KP_ORI*turnError + KI_ORI*totalTurnError + KD_ORI*(turnError - prevTurnError);
					totalTurnError += turnError;
					prevTurnError = turnError;
					
					if (fabs(tVel) < .25)
					{
						if (tVel < 0)
							tVel = -.25;
						else
							tVel = .25;
					}
				}
				//correct position
				else
				{
					totalTurnError = 0;
					float hErr = KP_POS*xError + KI_POS*totalXError
					+ KD_POS*(xError - prevXError);
					totalXError += xError;
					prevXError = xError;
					float vErr = KP_POS*yError + KI_POS*totalYError
					+ KD_POS*(yError - prevYError);
					totalYError += yError;
					prevYError = yError;
				
					xVel = hErr*cos(currentHeading) + vErr*sin(currentHeading);
					yVel = (-1*hErr*sin(currentHeading) + vErr*cos(currentHeading));
					//ROS_INFO("currentHeading: %f, hErr: %f, vErr: %f; xVel: %f, yVel: %f", currentHeading, hErr, vErr, xVel, yVel);
					//ROS_INFO("xError: %f, yError: %f", xError, yError);			
	}
			
				if (xVel > 1)
					xVel = 1;
				else if (xVel < -1)
					xVel = -1;
				if (yVel > 1)
					yVel = 1;
				else if (yVel < -1)
					yVel = -1;
				if (tVel > 1)
					tVel = 1;
				else if (tVel < -1)
					tVel = -1;
					
				//reduce speed for safety
				xVel =	xVel * .25;
				yVel = 	yVel * .25;
				tVel = tVel * .5;
		
				//publish to cmd_vel
				geometry_msgs::Twist baseCommand;
				baseCommand.linear.x = xVel;
				baseCommand.linear.y = yVel;
				baseCommand.angular.z = tVel;
				baseCommandPublisher.publish(baseCommand);
			
				r.sleep();
			}		
			
			ROS_INFO("Navigate action succeeded");
			asNavigateResult.result_msg = "Navigation succeeded";
			asNavigateResult.success = true;
			asNavigate.setSucceeded(asNavigateResult);
		}
		else
		{
			ROS_INFO("Navigate action succeeded");
			asNavigateResult.result_msg = "Navigation succeeded";
			asNavigateResult.success = true;
			asNavigate.setSucceeded(asNavigateResult);
		}
	}
	else
	{
		ROS_INFO("Navigate action failed");
		asNavigateResult.result_msg = "Navigation failed";
		asNavigateResult.success = false;
		asNavigate.setSucceeded(asNavigateResult);
	}
}

void taskActions::executeBackup(const retrieve_medicine::BackupGoalConstPtr& goal)
{
	//backup 1 meter
	ros::Rate r(60);

	for (int i = 0; i < 120; i ++)
	{
		geometry_msgs::Twist baseCommand;
		baseCommand.linear.x = -0.2;
		baseCommand.linear.y = 0;
		baseCommand.angular.z = 0;
		baseCommandPublisher.publish(baseCommand);
	
		r.sleep();
	}
	
	geometry_msgs::Twist baseCommand;
	baseCommand.linear.x = 0;
	baseCommand.linear.y = 0;
	baseCommand.angular.z = 0;
	baseCommandPublisher.publish(baseCommand);
	
	//reset torso height
	pr2_controllers_msgs::SingleJointPositionGoal torsoGoal;
	torsoGoal.position = 0.0;
	acMoveTorso.sendGoal(torsoGoal);
	acMoveTorso.waitForResult(ros::Duration(20));
	
	//tuck arms
	pr2_common_action_msgs::TuckArmsGoal armTuckGoal;
	armTuckGoal.tuck_left = true;
	armTuckGoal.tuck_right = true;
	acTuckArms.sendGoal(armTuckGoal);
	acTuckArms.waitForResult(ros::Duration(15));
	
	ROS_INFO("Backup action succeeded");
	asBackupResult.result_msg = "Backup succeeded";
	asBackupResult.success = true;
	asBackup.setSucceeded(asBackupResult);
}

void taskActions::executePickupAll(const retrieve_medicine::PickupAllGoalConstPtr& goal)
{
	//open grippers
	pr2_controllers_msgs::Pr2GripperCommandGoal openGripper;
	openGripper.command.position = 0.08;
	openGripper.command.max_effort = -1.0;
	acLeftGripper.sendGoal(openGripper);
	acRightGripper.sendGoal(openGripper);
	
	//center head on table
	pr2_interactive_object_detection::UserCommandGoal segmentGoal;
	segmentGoal.request = 0;	//look at table
	acSegment.sendGoal(segmentGoal);
	acSegment.waitForResult(ros::Duration(15));
	
	//segment
	resetCollisionObjects();
	segmentGoal.request = 1;	//segment
	acSegment.sendGoal(segmentGoal);
	acSegment.waitForResult(ros::Duration(15));
	
	ros::Rate segmentRate(10);
	while (hasSegmented == false)
	{
		segmentRate.sleep();
	}
	
	//pickup objects
	if (objectList.graspable_objects.size() > 0)
	{	
		hasSegmented = false;
		ROS_INFO("Picking up object...");
		pr2_object_manipulation_msgs::IMGUIGoal imguiGoal;
		pr2_object_manipulation_msgs::IMGUIOptions imguiOptions;
	
		imguiOptions.collision_checked = true;
		imguiOptions.grasp_selection = 1;	//grasp provided object
		imguiOptions.arm_selection = 0;	//TODO set this based on object position
		imguiOptions.selected_object = objectList.graspable_objects[0]; //selected object
		
		imguiGoal.command.command = 0; 	//pickup
		imguiGoal.options = imguiOptions;
		
		acIMGUI.sendGoal(imguiGoal);
		
		acIMGUI.waitForResult();
		
		if (acIMGUI.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_INFO("Pickup succeeded, checking grasp...");
			//check for successful grasp
			interactive_world_hackathon::GraspCheck srv;
			srv.request.side = "right";
			graspCheckClient.call(srv);
			if (!srv.response.isGrasping)
			{
				ROS_INFO("Grasp failed");
				ROS_INFO("Pickup All action failed");
				asPickupAllResult.result_msg = "Pickup All failed";
				asPickupAllResult.success = false;
				asPickupAll.setSucceeded(asPickupAllResult);
				return;
			}	
			
			ROS_INFO("Grasp succeeded!");
			//move arm to side
			pr2_object_manipulation_msgs::IMGUIGoal imguiGoal;
			pr2_object_manipulation_msgs::IMGUIOptions imguiOptions;
	
			imguiOptions.collision_checked = true;
			imguiOptions.grasp_selection = 0;	//grasp provided object
			imguiOptions.arm_selection = 0;	//TODO set this based on object position
			imguiOptions.arm_action_choice = 0;	//move arm to side
			imguiOptions.arm_planner_choice = 1;	//move arm with planner
		
			imguiGoal.command.command = 4; 	//move arm
			imguiGoal.options = imguiOptions;
		
			acIMGUI.sendGoal(imguiGoal);
		
			acIMGUI.waitForResult();
			
			if (acIMGUI.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				//segment
				resetCollisionObjects();
				segmentGoal.request = 1;	//segment
				acSegment.sendGoal(segmentGoal);
				acSegment.waitForResult(ros::Duration(15));
				
				while (hasSegmented == false)
				{
					segmentRate.sleep();
				}
				
				//pickup objects
				if (objectList.graspable_objects.size() > 0)
				{	
					hasSegmented = false;
					ROS_INFO("Picking up object...");
					pr2_object_manipulation_msgs::IMGUIGoal imguiGoal;
					pr2_object_manipulation_msgs::IMGUIOptions imguiOptions;
				
					imguiOptions.collision_checked = true;
					imguiOptions.grasp_selection = 1;	//grasp provided object
					imguiOptions.arm_selection = 1;	//TODO set this based on object position
					imguiOptions.selected_object = objectList.graspable_objects[0]; //selected object
		
					imguiGoal.command.command = 0; 	//pickup
					imguiGoal.options = imguiOptions;
		
					acIMGUI.sendGoal(imguiGoal);
		
					acIMGUI.waitForResult();
		
					if (acIMGUI.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
					{
						ROS_INFO("Pickup succeeded, checking grasp...");
						//check for successful grasp
						interactive_world_hackathon::GraspCheck srv;
						srv.request.side = "left";
						graspCheckClient.call(srv);
						if (!srv.response.isGrasping)
						{
							ROS_INFO("Grasp failed");
							ROS_INFO("Pickup All action failed");
							asPickupAllResult.result_msg = "Pickup All failed";
							asPickupAllResult.success = false;
							asPickupAll.setSucceeded(asPickupAllResult);
							return;
						}
						
						ROS_INFO("Grasp succeeded!");
						//move arm to side
						pr2_object_manipulation_msgs::IMGUIGoal imguiGoal;
						pr2_object_manipulation_msgs::IMGUIOptions imguiOptions;
	
						imguiOptions.collision_checked = true;
						imguiOptions.grasp_selection = 0;	//grasp provided object
						imguiOptions.arm_selection = 1;	//TODO set this based on object position
						imguiOptions.arm_action_choice = 0;	//move arm to side
						imguiOptions.arm_planner_choice = 1;	//move arm with planner
		
						imguiGoal.command.command = 4; 	//move arm
						imguiGoal.options = imguiOptions;
		
						acIMGUI.sendGoal(imguiGoal);
		
						acIMGUI.waitForResult();
			
						if (acIMGUI.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
						{
							ROS_INFO("Pickup All action succeeded");
							asPickupAllResult.result_msg = "Pickup All succeeded";
							asPickupAllResult.success = true;
							asPickupAll.setSucceeded(asPickupAllResult);
							return;
						}
					}
				}
			}
		}
	}
	
	ROS_INFO("Pickup All action failed");
	asPickupAllResult.result_msg = "Pickup All failed";
	asPickupAllResult.success = false;
	asPickupAll.setSucceeded(asPickupAllResult);
}

void taskActions::resetCollisionObjects()
{
	pr2_object_manipulation_msgs::IMGUIGoal imguiGoal;
	pr2_object_manipulation_msgs::IMGUIOptions imguiOptions;
	imguiOptions.collision_checked = true;
	imguiOptions.grasp_selection = 0;
	imguiOptions.reset_choice = 0;	//reset collision objects
	
	imguiGoal.command.command = 3; 	//reset collision objects
	imguiGoal.options = imguiOptions;

	acIMGUI.sendGoal(imguiGoal);

	acIMGUI.waitForResult();
}

void taskActions::basePoseCallback(const geometry_msgs::Pose& newPose)
{
	basePose.position.x = newPose.position.x;
	basePose.position.y = newPose.position.y;
	basePose.position.z = newPose.position.z;
	basePose.orientation.x = newPose.orientation.x;
	basePose.orientation.y = newPose.orientation.y;
	basePose.orientation.z = newPose.orientation.z;
	basePose.orientation.w = newPose.orientation.w;
}

void taskActions::objectCallback(const manipulation_msgs::GraspableObjectList& objects)
{
	ROS_INFO("Received new object data");
	objectList = objects;
	ROS_INFO("Updated object data");
	hasSegmented = true;
}

void taskActions::executeHandoff(const retrieve_medicine::handoffGoalConstPtr& goal)
{
	
	//Handoff arms to the person
	pr2_controllers_msgs::JointTrajectoryGoal leftArmGoal;
	leftArmGoal.trajectory.joint_names = leftArmJointNames;
	trajectory_msgs::JointTrajectoryPoint leftHandoffPoint;
	leftHandoffPoint.positions = leftArmHandoffPosition;
	leftHandoffPoint.time_from_start = ros::Duration(3);
	leftArmGoal.trajectory.points.push_back(leftHandoffPoint);

	pr2_controllers_msgs::JointTrajectoryGoal rightArmGoal;
	rightArmGoal.trajectory.joint_names = rightArmJointNames;
	trajectory_msgs::JointTrajectoryPoint rightHandoffPoint;
	rightHandoffPoint.positions = rightArmHandoffPosition;
	rightHandoffPoint.time_from_start = ros::Duration(3);
	rightArmGoal.trajectory.points.push_back(rightHandoffPoint);

	acLeftArm.sendGoal(leftArmGoal);
	acRightArm.sendGoal(rightArmGoal);
	acLeftArm.waitForResult(ros::Duration(6));
	acRightArm.waitForResult(ros::Duration(6));

	//
}

void taskActions::executeRelease(const retrieve_medicine::ReleaseGoalConstPtr& goal)
{
	ROS_INFO("Entered release callback...");

	//listen to transform coordinates and detect the change
	tf::TransformListener listener;
	float preLeftGripperX, preLeftGripperY, preRightGripperX, preRightGripperY;
	int count = 0;
  	
	preLeftGripperX = 0.0;
	preLeftGripperY = 0.0;
	preRightGripperX = 0.0;
	preRightGripperY = 0.0;

	bool lflag, rflag;
	lflag = false;
	rflag = false;
	
	ros::Rate r(20);

	while (1){
		r.sleep();
		ROS_INFO("Looking up transforms");
		count++;
	        tf::StampedTransform leftGripperTransform;
	        tf::StampedTransform rightGripperTransorm;
		try{
				listener.waitForTransform("/base_link", "/l_gripper_palm_link", ros::Time(0), ros::Duration(3.0));
      			listener.lookupTransform("/base_link", "/l_gripper_palm_link",  
                            			   ros::Time(0), leftGripperTransform);
			listener.lookupTransform("/base_link", "/r_gripper_palm_link",  
                              			 ros::Time(0), rightGripperTransorm);
		    }
    		catch (tf::TransformException ex){
		         ROS_ERROR("%s",ex.what());
                    }
		//ROS_INFO("Left gripper TF difference: %f", fabs( leftGripperTransform.getOrigin().x() - preLeftGripperX ));
		//ROS_INFO("Right gripper TF difference: %f", fabs( leftGripperTransform.getOrigin().y() - preLeftGripperY ));
		//ROS_INFO("count: %d", count);

	  //open left gripper if the coordinate frame of left gripper changes
	  if( !lflag && count > 50 && fabs( leftGripperTransform.getOrigin().x() - preLeftGripperX ) > 0.0003 && fabs( leftGripperTransform.getOrigin().y() - preLeftGripperY ) > 0.0003 )
	  {
	  	ROS_INFO("Releasing left gripper");
		pr2_controllers_msgs::Pr2GripperCommandGoal openGripper;
		openGripper.command.position = 0.08;
		openGripper.command.max_effort = -1.0;
		acLeftGripper.sendGoal(openGripper);
		acLeftGripper.waitForResult(ros::Duration(3));
		lflag = true;
	  }

	  //open right gripper if the coordinate frame of right gripper changes
          if( !rflag && count > 50 && fabs( rightGripperTransorm.getOrigin().x() - preRightGripperX ) > 0.0003 && fabs( rightGripperTransorm.getOrigin().y() - preRightGripperY ) > 0.0003 )
	  {
		  ROS_INFO("Releasing right gripper");
		pr2_controllers_msgs::Pr2GripperCommandGoal openGripper;
		openGripper.command.position = 0.08;
		openGripper.command.max_effort = -1.0;
		acRightGripper.sendGoal(openGripper);
		acRightGripper.waitForResult(ros::Duration(3));
		rflag = true;
	  }

	  if( lflag && rflag) break;
	  
          preLeftGripperX = leftGripperTransform.getOrigin().x();
	  preLeftGripperY = leftGripperTransform.getOrigin().y();
	  preRightGripperX = rightGripperTransorm.getOrigin().x();
	  preRightGripperY = rightGripperTransorm.getOrigin().y();
	  
	  
	}

	ROS_INFO("Release action succeeded");
	asReleaseResult.result_msg = "Release succeeded";
	asReleaseResult.success = true;
	asRelease.setSucceeded(asReleaseResult);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "task_actions");

    taskActions rm;

    ros::spin();    

    return 0;
}


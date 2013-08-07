#include <retrieve_medicine/retrieve_medicine.h>

using namespace std;

retrieveMedicine::retrieveMedicine(string name) : 
	acMoveBase("/move_base", true),
	acMoveHead("/head_traj_controller/point_head_action", true),
	acMoveTorso("/torso_controller/position_joint_action", true),
	acLeftArm("/l_arm_controller/joint_trajectory_action", true),
	acRightArm("/r_arm_controller/joint_trajectory_action", true),
	acLeftGripper("/l_gripper_controller/gripper_action", true),
	acRightGripper("/r_gripper_controller/gripper_action", true),
	acSegment("/object_detection_user_command", true),
	acTuckArms("/tuck_arms", true),
	asNavigate(n, name, boost::bind(&retrieveMedicine::executeNavigate, this, _1), false),
actionName(name)
{
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

    asNavigate.start();

	baseCommandPublisher = n.advertise<geometry_msgs::Twist>("/base_controller/command", -1);

    position_client = n.serviceClient<position_server::GetPosition>("position_server/get_position");
    
    //Define arm joint positions
	string rightJoints[] = {"r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", "r_wrist_roll_joint"};
	string leftJoints[] = {"l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"};
	leftArmJointNames.assign(leftJoints, leftJoints + 7);
	rightArmJointNames.assign(rightJoints, rightJoints + 7);
	double leftSidePos[] = {2.115, 0.0, 1.64, -2.07, 1.64, -1.680, 1.398};
	double rightSidePos[] = {-2.115, 0.0, -1.64, -2.07, -1.64, -1.680, 1.398};
	leftArmSidePosition.assign(leftSidePos, leftSidePos + 7);
	rightArmSidePosition.assign(rightSidePos, rightSidePos + 7);
	
	basePoseSubscriber = n.subscribe("/robot_pose", 1, &retrieveMedicine::basePoseCallback, this);
}

void retrieveMedicine::executeNavigate(const retrieve_medicine::navigateGoalConstPtr& goal)
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

	ROS_INFO("Tucking arms");

	pr2_common_action_msgs::TuckArmsGoal armTuckGoal;
	armTuckGoal.tuck_left = true;
	armTuckGoal.tuck_right = true;
	acTuckArms.sendGoal(armTuckGoal);
	acTuckArms.waitForResult(ros::Duration(15));

	ROS_INFO("Navigating to requested position");
	
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
	moveGoal.target_pose = target;
	acMoveBase.sendGoal(moveGoal);

	acMoveBase.waitForResult(ros::Duration(30));

	if (acMoveBase.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("Untuck arms and adjust height");
		//Untuck arms and set torso height
		pr2_common_action_msgs::TuckArmsGoal armUntuckGoal;
		armUntuckGoal.tuck_left = false;
		armUntuckGoal.tuck_right = false;
		pr2_controllers_msgs::SingleJointPositionGoal torsoGoal;
		torsoGoal.position = srv.response.position.height;
		if (torsoGoal.position < 0.0)
			torsoGoal.position = 0.0;
		else if (torsoGoal.position > 0.6)
			torsoGoal.position = 0.6;
		acTuckArms.sendGoal(armUntuckGoal);
		acMoveTorso.sendGoal(torsoGoal);
		acTuckArms.waitForResult(ros::Duration(20));
		
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
			
			float currentHeading = tf::getYaw(basePose.orientation);
			
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
				currentHeading = tf::getYaw(basePose.orientation);
				
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
					//ROS_INFO("hErr: %f, vErr: %f; xVel: %f, yVel: %f", hErr, vErr, xVel, yVel);
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
			
			ROS_INFO("%s: Succeeded complete", actionName.c_str());
			asNavigateResult.result_msg = "Finished";
			asNavigateResult.success = true;
			asNavigate.setSucceeded(asNavigateResult);
		}
		else
		{
			ROS_INFO("%s: Succeeded complete", actionName.c_str());
			asNavigateResult.result_msg = "Finished";
			asNavigateResult.success = true;
			asNavigate.setSucceeded(asNavigateResult);
		}
	}
	else
	{
		ROS_INFO("%s: Failed to complete action", actionName.c_str());
		asNavigate.setPreempted();
	}
	
}

void retrieveMedicine::basePoseCallback(const geometry_msgs::Pose& newPose)
{
	basePose.position.x = newPose.position.x;
	basePose.position.y = newPose.position.y;
	basePose.position.z = newPose.position.z;
	basePose.orientation.x = newPose.orientation.x;
	basePose.orientation.y = newPose.orientation.y;
	basePose.orientation.z = newPose.orientation.z;
	basePose.orientation.w = newPose.orientation.w;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "retrieve_medicine");

    retrieveMedicine rm("navigate_action");

    ros::spin();    

    return 0;
}


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
	as(n, name, boost::bind(&retrieveMedicine::executeNavigate, this, _1), false),
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

    as.start();

    position_client = n.serviceClient<position_server::GetPosition>("position_server/get_position");
}


void retrieveMedicine::executeNavigate(const retrieve_medicine::navigateGoalConstPtr& goal)
{
	ROS_INFO("Goal task name: %s", goal->taskName.c_str());
	
	position_server::GetPosition srv;
        srv.request.positionName = goal->taskName;

	pr2_common_action_msgs::TuckArmsGoal armTuckGoal;
	armTuckGoal.tuck_left = true;
	armTuckGoal.tuck_right = true;
	acTuckArms.sendGoal(armTuckGoal);
	acTuckArms.waitForResult(ros::Duration(15));

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
		pr2_common_action_msgs::TuckArmsGoal armUntuckGoal;
		armUntuckGoal.tuck_left = false;
		armUntuckGoal.tuck_right = false;
		acTuckArms.sendGoal(armUntuckGoal);
		acTuckArms.waitForResult(ros::Duration(30));

		ROS_INFO("%s: Succeeded complete", actionName.c_str());
		asResult.result_msg = "Finished";
		asResult.success = true;
		as.setSucceeded(asResult);
	}
	else
	{
		ROS_INFO("%s: Failed to complete action", actionName.c_str());
		as.setPreempted();
	}
	
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "retrieve_medicine");

    retrieveMedicine rm("navigate_action");

    ros::spin();    

    return 0;
}


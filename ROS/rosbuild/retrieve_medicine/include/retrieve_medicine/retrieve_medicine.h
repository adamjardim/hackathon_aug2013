#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <pr2_common_action_msgs/TuckArmsAction.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <retrieve_medicine/navigateAction.h>
#include <retrieve_medicine/handoffAction.h>
#include <pr2_controllers_msgs/SingleJointPositionAction.h>
#include <pr2_interactive_object_detection/UserCommandAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <position_server/GetPosition.h>

class retrieveMedicine
{
public: 
	ros::NodeHandle n;    

	ros::ServiceClient position_client;

	//Action clients
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> acMoveBase;
	actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> acMoveHead;
	actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction> acMoveTorso;
	actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> acLeftArm;
	actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> acRightArm;
	actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> acLeftGripper;
	actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> acRightGripper;
	actionlib::SimpleActionClient<pr2_interactive_object_detection::UserCommandAction> acSegment;
	actionlib::SimpleActionClient<pr2_common_action_msgs::TuckArmsAction> acTuckArms;

	actionlib::SimpleActionServer<retrieve_medicine::navigateAction> asNavigate;
	actionlib::SimpleActionServer<retrieve_medicine::handoffAction> asHandoff;
	
	std::string actionName;
    
	retrieve_medicine::navigateFeedback asNavigateFeedback;
	retrieve_medicine::navigateResult asNavigateResult;
    
	retrieve_medicine::handoffFeedback asHandoffFeedback;
	retrieve_medicine::handoffResult asHandoffResult;

	//Arm positions
	std::vector<double> leftArmSidePosition;
	std::vector<double> rightArmSidePosition;

	std::vector<double> leftArmHandoffPosition;
	std::vector<double> rightArmHandoffPosition;

	std::vector<std::string> leftArmJointNames;
	std::vector<std::string> rightArmJointNames;

	retrieveMedicine(std::string name);

	void executeNavigate(const retrieve_medicine::navigateGoalConstPtr& goal);
	void executeHandoff(const retrieve_medicine::handoffGoalConstPtr& goal);
};


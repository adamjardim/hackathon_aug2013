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
#include <retrieve_medicine/BackupAction.h>
#include <retrieve_medicine/PickupAllAction.h>
#include <retrieve_medicine/ReleaseAction.h>
#include <pr2_controllers_msgs/SingleJointPositionAction.h>
#include <pr2_interactive_object_detection/UserCommandAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <position_server/GetPosition.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <pr2_object_manipulation_msgs/IMGUIAction.h>
//#include <pr2_object_manipulation_msgs/IMGUICommand.h>
//#include <pr2_object_manipulation_msgs/IMGUIOptions.h>

#define KP_POS 4
#define KI_POS .001
#define KD_POS .01
#define KP_ORI 4.3
#define KI_ORI .01
#define KD_ORI .008

#define PI 3.14159

struct vec3
{
	float x, y, t;
};

class taskActions
{
public: 
    ros::NodeHandle n;
    
    ros::Publisher baseCommandPublisher;
    ros::Subscriber basePoseSubscriber;

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
	actionlib::SimpleActionClient<pr2_object_manipulation_msgs::IMGUIAction> acIMGUI;

	//Action servers
	actionlib::SimpleActionServer<retrieve_medicine::navigateAction> asNavigate;
	actionlib::SimpleActionServer<retrieve_medicine::handoffAction> asHandoff;
	actionlib::SimpleActionServer<retrieve_medicine::BackupAction> asBackup;
	actionlib::SimpleActionServer<retrieve_medicine::PickupAllAction> asPickupAll;
	actionlib::SimpleActionServer<retrieve_medicine::ReleaseAction> asRelease;
    
	retrieve_medicine::navigateFeedback asNavigateFeedback;
	retrieve_medicine::navigateResult asNavigateResult;
    
	retrieve_medicine::handoffFeedback asHandoffFeedback;
	retrieve_medicine::handoffResult asHandoffResult;
	
	retrieve_medicine::BackupFeedback asBackupFeedback;
	retrieve_medicine::BackupResult asBackupResult;
	
	retrieve_medicine::PickupAllFeedback asPickupAllFeedback;
	retrieve_medicine::PickupAllResult asPickupAllResult;

	retrieve_medicine::ReleaseFeedback asReleaseFeedback;
	retrieve_medicine::ReleaseResult asReleaseResult;

	//Arm positions
	std::vector<double> leftArmSidePosition;
	std::vector<double> rightArmSidePosition;

	std::vector<double> leftArmHandoffPosition;
	std::vector<double> rightArmHandoffPosition;

	std::vector<std::string> leftArmJointNames;
	std::vector<std::string> rightArmJointNames;
	
	geometry_msgs::Pose basePose;

	taskActions();

        void executeNavigate(const retrieve_medicine::navigateGoalConstPtr& goal);
    
	void executeBackup(const retrieve_medicine::BackupGoalConstPtr& goal);
    
	void executePickupAll(const retrieve_medicine::PickupAllGoalConstPtr& goal);
    
	void basePoseCallback(const geometry_msgs::Pose& newPose);
    
	void executeHandoff(const retrieve_medicine::handoffGoalConstPtr& goal);

	void executeRelease(const retrieve_medicine::ReleaseGoalConstPtr& goal);
};


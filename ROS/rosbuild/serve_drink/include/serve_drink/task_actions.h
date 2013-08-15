#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <pr2_common_action_msgs/TuckArmsAction.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <serve_drink/navigateAction.h>
#include <serve_drink/handoffAction.h>
#include <serve_drink/BackupAction.h>
#include <serve_drink/PickupAllAction.h>
#include <serve_drink/ReleaseAction.h>
#include <pr2_controllers_msgs/SingleJointPositionAction.h>
#include <pr2_interactive_object_detection/UserCommandAction.h>
#include <manipulation_msgs/GraspableObjectList.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <position_server/GetPosition.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <pr2_object_manipulation_msgs/IMGUIAction.h>
#include <interactive_world_hackathon/GraspCheck.h>
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
    ros::Subscriber objectSubscriber;

	ros::ServiceClient position_client;
	ros::ServiceClient graspCheckClient;

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
	actionlib::SimpleActionServer<serve_drink::navigateAction> asNavigate;
	actionlib::SimpleActionServer<serve_drink::handoffAction> asHandoff;
	actionlib::SimpleActionServer<serve_drink::BackupAction> asBackup;
	actionlib::SimpleActionServer<serve_drink::PickupAllAction> asPickupAll;
	actionlib::SimpleActionServer<serve_drink::ReleaseAction> asRelease;
    
	serve_drink::navigateFeedback asNavigateFeedback;
	serve_drink::navigateResult asNavigateResult;
    
	serve_drink::handoffFeedback asHandoffFeedback;
	serve_drink::handoffResult asHandoffResult;
	
	serve_drink::BackupFeedback asBackupFeedback;
	serve_drink::BackupResult asBackupResult;
	
	serve_drink::PickupAllFeedback asPickupAllFeedback;
	serve_drink::PickupAllResult asPickupAllResult;

	serve_drink::ReleaseFeedback asReleaseFeedback;
	serve_drink::ReleaseResult asReleaseResult;

	//Arm positions
	std::vector<double> leftArmSidePosition;
	std::vector<double> rightArmSidePosition;

	std::vector<double> leftArmHandoffPosition;
	std::vector<double> rightArmHandoffPosition;

	std::vector<std::string> leftArmJointNames;
	std::vector<std::string> rightArmJointNames;
	
	geometry_msgs::Pose basePose;
	
	//segmented objects
	manipulation_msgs::GraspableObjectList objectList;
	bool hasSegmented;

	taskActions();

  void executeNavigate(const serve_drink::navigateGoalConstPtr& goal);
    
	void executeBackup(const serve_drink::BackupGoalConstPtr& goal);
    
	void executePickupAll(const serve_drink::PickupAllGoalConstPtr& goal);
    
	void basePoseCallback(const geometry_msgs::Pose& newPose);
    
	void executeHandoff(const serve_drink::handoffGoalConstPtr& goal);

	void executeRelease(const serve_drink::ReleaseGoalConstPtr& goal);
	
	void objectCallback(const manipulation_msgs::GraspableObjectList& objects);

	void resetCollisionObjects();
};


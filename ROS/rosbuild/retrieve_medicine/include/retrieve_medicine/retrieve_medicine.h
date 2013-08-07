#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <pr2_common_action_msgs/TuckArmsAction.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <retrieve_medicine/navigateAction.h>
#include <pr2_controllers_msgs/SingleJointPositionAction.h>
#include <pr2_interactive_object_detection/UserCommandAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <position_server/GetPosition.h>
#include <tf/tf.h>

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

class retrieveMedicine
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
	actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> acRightGripper;	actionlib::SimpleActionClient<pr2_interactive_object_detection::UserCommandAction> acSegment;
	actionlib::SimpleActionClient<pr2_common_action_msgs::TuckArmsAction> acTuckArms;

    actionlib::SimpleActionServer<retrieve_medicine::navigateAction> asNavigate;
    std::string actionName;
    retrieve_medicine::navigateFeedback asNavigateFeedback;
    retrieve_medicine::navigateResult asNavigateResult;

	//Arm positions
	std::vector<double> leftArmSidePosition;
	std::vector<double> rightArmSidePosition;
	std::vector<std::string> leftArmJointNames;
	std::vector<std::string> rightArmJointNames;
	
	geometry_msgs::Pose basePose;

    retrieveMedicine(std::string name);

    void executeNavigate(const retrieve_medicine::navigateGoalConstPtr& goal);
    
    void basePoseCallback(const geometry_msgs::Pose& newPose);
};


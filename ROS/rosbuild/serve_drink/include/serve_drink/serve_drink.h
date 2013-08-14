#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <serve_drink/navigateAction.h>
#include <serve_drink/handoffAction.h>
#include <serve_drink/BackupAction.h>
#include <serve_drink/PickupAllAction.h>
#include <serve_drink/ServeDrinkAction.h>
#include <serve_drink/ReleaseAction.h>

//State
#define STATE_NAVIGATION_1 1
#define STATE_PICKUP 2
#define STATE_BACKUP 3
#define STATE_NAVIGATION_2 4
#define STATE_HANDOFF 5

class serveDrink
{
public:
    ros::NodeHandle n;
    
    int state;
    
    //Action clients
	actionlib::SimpleActionClient<serve_drink::navigateAction> acNavigate;
	actionlib::SimpleActionClient<serve_drink::handoffAction> acHandoff;
	actionlib::SimpleActionClient<serve_drink::BackupAction> acBackup;
	actionlib::SimpleActionClient<serve_drink::PickupAllAction> acPickupAll;
	actionlib::SimpleActionClient<serve_drink::ReleaseAction> acRelease;

	//Action servers
	actionlib::SimpleActionServer<serve_drink::ServeDrinkAction> asServeDrink;
	
	serve_drink::ServeDrinkFeedback asServeDrinkFeedback;
	serve_drink::ServeDrinkResult asServeDrinkResult;

	serveDrink();

    void executeServeDrink(const serve_drink::ServeDrinkGoalConstPtr& goal);
};


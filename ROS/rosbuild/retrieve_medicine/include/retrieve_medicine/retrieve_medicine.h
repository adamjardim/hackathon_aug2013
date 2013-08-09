#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <retrieve_medicine/navigateAction.h>
#include <retrieve_medicine/handoffAction.h>
#include <retrieve_medicine/BackupAction.h>
#include <retrieve_medicine/PickupAllAction.h>
#include <retrieve_medicine/RetrieveMedicineAction.h>
#include <retrieve_medicine/ReleaseAction.h>

//State
#define STATE_NAVIGATION_1 1
#define STATE_PICKUP 2
#define STATE_BACKUP 3
#define STATE_NAVIGATION_2 4
#define STATE_HANDOFF 5

class retrieveMedicine
{
public:
    ros::NodeHandle n;
    
    int state;
    
    //Action clients
	actionlib::SimpleActionClient<retrieve_medicine::navigateAction> acNavigate;
	actionlib::SimpleActionClient<retrieve_medicine::handoffAction> acHandoff;
	actionlib::SimpleActionClient<retrieve_medicine::BackupAction> acBackup;
	actionlib::SimpleActionClient<retrieve_medicine::PickupAllAction> acPickupAll;
	actionlib::SimpleActionClient<retrieve_medicine::ReleaseAction> acRelease;

	//Action servers
	actionlib::SimpleActionServer<retrieve_medicine::RetrieveMedicineAction> asRetrieveMedicine;
	
	retrieve_medicine::RetrieveMedicineFeedback asRetrieveMedicineFeedback;
	retrieve_medicine::RetrieveMedicineResult asRetrieveMedicineResult;

	retrieveMedicine();

    void executeRetrieveMedicine(const retrieve_medicine::RetrieveMedicineGoalConstPtr& goal);
};


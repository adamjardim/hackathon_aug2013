#include <serve_drink/serve_drink.h>

using namespace std;

serveDrink::serveDrink() : 
	acNavigate("/navigate_action", true),
	acHandoff("/handoff_action", true),
	acBackup("/backup_action", true),
	acPickupAll("/pickup_all_action", true),
	acRelease("/release_action", true),
    acHighfive("/high_five", true),
	asServeDrink(n, "serve_drink_action", boost::bind(&serveDrink::executeServeDrink, this, _1), false)
{
	ROS_INFO("Waiting for navigate action server...");
	acNavigate.waitForServer();
	ROS_INFO("Finished waiting for navigate action server.");
	
	ROS_INFO("Waiting for handoff action server...");
	acHandoff.waitForServer();
	ROS_INFO("Finished waiting for handoff action server.");
	
	ROS_INFO("Waiting for backup action server...");
	acBackup.waitForServer();
	ROS_INFO("Finished waiting for backup action server.");
	
    ROS_INFO("Waiting for pickup all action server...string");
	acPickupAll.waitForServer();
	ROS_INFO("Finished waiting for pickup all action server.");

	asServeDrink.start();
	
    state = STATE_NAVIGATION_0;
}

bool serveDrink::executeNavigate(string dest, bool align, int nextState)
{
    serve_drink::navigateGoal navGoal;
    navGoal.taskName = dest;
    navGoal.align = align;
    acNavigate.sendGoal(navGoal);
    acNavigate.waitForResult();
    serve_drink::navigateResultConstPtr navResult = acNavigate.getResult();

    if (navResult->success == false)
    {
        state = nextState;
        ROS_INFO("Navigate to %s action failed", dest.c_str());
        asServeDrinkResult.result_msg = "Navigation to " + dest + "failed, please complete the task manually.";
        asServeDrinkResult.success = false;
        asServeDrink.setSucceeded(asServeDrinkResult);
        return false;
    }

    state = nextState;

    return true;
}

bool serveDrink::executeHighfive(int nextState)
{
    pr2_props::HighFiveGoal highfiveGoal;
    acHighfive.sendGoal(highfiveGoal);
    acHighfive.waitForResult();
    pr2_props::HighFiveResultConstPtr highfiveResult = acHighfive.getResult();

    if (highfiveResult->success == false)
    {
        state = nextState;
        ROS_INFO("Highfive action failed");
        asServeDrinkResult.result_msg = "Highfive action failed, please complete the task manually.";
        asServeDrinkResult.success = false;
        asServeDrink.setSucceeded(asServeDrinkResult);
        return false;
    }

    state = nextState;

    return true;
}


bool serveDrink::executePickup(int model_id, int nextState)
{
    serve_drink::PickupAllGoal pickupGoal;
    pickupGoal.model_id = model_id;
    acPickupAll.sendGoal(pickupGoal);
    acPickupAll.waitForResult();
    serve_drink::PickupAllResultConstPtr pickupResult = acPickupAll.getResult();

    if (pickupResult->success == false)
    {
        state = nextState;
        ROS_INFO("Pickup %d action failed", model_id);
        asServeDrinkResult.result_msg = "pickup failed, please complete the task manually.";
        asServeDrinkResult.success = false;
        asServeDrink.setSucceeded(asServeDrinkResult);
        return false;
    }

    state = nextState;

    return true;
}

bool serveDrink::executeBackup(int nextState)
{
    serve_drink::BackupGoal backupGoal;
    acBackup.sendGoal(backupGoal);
    acBackup.waitForResult();
    serve_drink::BackupResultConstPtr backupResult = acBackup.getResult();

    if (backupResult->success == false)
    {
        state = nextState;
        ROS_INFO("Backup action failed");
        asServeDrinkResult.result_msg = "Backup failed, please complete the task manually.";
        asServeDrinkResult.success = false;
        asServeDrink.setSucceeded(asServeDrinkResult);
        return false;
    }

    state = nextState;

    return true;
}

bool serveDrink::executeRelease(int nextState)
{
    serve_drink::ReleaseGoal releaseGoal;
    acRelease.sendGoal(releaseGoal);
    acRelease.waitForResult();
    serve_drink::ReleaseResultConstPtr releaseResult = acRelease.getResult();

    if (releaseResult->success == false)
    {
        state = nextState;
        ROS_INFO("Handoff action failed");
        asServeDrinkResult.result_msg = "Handoff action failed, please complete the task manually.";
        asServeDrinkResult.success = false;
        asServeDrink.setSucceeded(asServeDrinkResult);
        return false;
    }

    state = nextState;

    return true;
}


void serveDrink::executeServeDrink(const serve_drink::ServeDrinkGoalConstPtr& goal)
{
    if( state == STATE_NAVIGATION_0 )
    {
        if( !executeNavigate("Stage Front", false, STATE_HIGHFIVE)) return;
    }

    if( state == STATE_HIGHFIVE )
    {
        if( !executeHighfive(STATE_NAVIGATION_1)) return;
    }

    if( state == STATE_NAVIGATION_1 )
    {
        if ( !executeNavigate("Serve Table Nav", true, STATE_PICKUP) ) return;
    }

    if( state == STATE_PICKUP )
    {
        if ( !executePickup(18783, STATE_BACKUP) ) return;
    }

    if( state == STATE_NAVIGATION_2 )
    {
        if( !executeNavigate("Stage Front", false, STATE_HANDOFF) ) return;
    }

    if( state == STATE_HANDOFF )
    {
        if( !executeRelease(STATE_NAVIGATION_0) ) return;
    }

    ROS_INFO("Serve Drink action succeeded");
    asServeDrinkResult.result_msg = "Serve Drink action succeeded";
    asServeDrinkResult.success = true;
    asServeDrink.setSucceeded(asServeDrinkResult);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serve_drink");

    serveDrink sd;

    ros::spin();

    return 0;
}


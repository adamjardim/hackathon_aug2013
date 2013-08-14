#include <serve_drink/serve_drink.h>

using namespace std;

serveDrink::serveDrink() : 
	acNavigate("/navigate_action", true),
	acHandoff("/handoff_action", true),
	acBackup("/backup_action", true),
	acPickupAll("/pickup_all_action", true),
	acRelease("/release_action", true),
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
	
	ROS_INFO("Waiting for pickup all action server...");
	acPickupAll.waitForServer();
	ROS_INFO("Finished waiting for pickup all action server.");

	asServeDrink.start();
	
	state = STATE_NAVIGATION_1;
}

void serveDrink::executeServeDrink(const serve_drink::ServeDrinkGoalConstPtr& goal)
{
	//Navigate to medicine counter and align to counter
	if (state == STATE_NAVIGATION_1)
	{
		serve_drink::navigateGoal navGoal;
		navGoal.taskName = "Serve Table Nav";
		navGoal.align = true;
		acNavigate.sendGoal(navGoal);
		acNavigate.waitForResult();
		serve_drink::navigateResultConstPtr navResult = acNavigate.getResult();

		if (navResult->success == false)
		{
			state = STATE_PICKUP;
			ROS_INFO("Navigate to Medicine Counter action failed");
			asServeDrinkResult.result_msg = "Navigation to medicine counter failed, please complete the task manually.";
			asServeDrinkResult.success = false;
			asServeDrink.setSucceeded(asServeDrinkResult);
			return;
		}
		
		state = STATE_PICKUP;
	}
	
	//Pickup medicine and water from counter
	if (state == STATE_PICKUP)
	{
		serve_drink::PickupAllGoal pickupGoal;
		acPickupAll.sendGoal(pickupGoal);
		acPickupAll.waitForResult();
		serve_drink::PickupAllResultConstPtr pickupResult = acPickupAll.getResult();

		if (pickupResult->success == false)
		{
			state = STATE_BACKUP;
			ROS_INFO("Pickup Medicine and Water action failed");
			asServeDrinkResult.result_msg = "Medicine and water pickup failed, please complete the task manually.";
			asServeDrinkResult.success = false;
			asServeDrink.setSucceeded(asServeDrinkResult);
			return;
		}
		
		state = STATE_BACKUP;
	}
	
	//Backup from medicine counter and tuck arms
	if (state == STATE_BACKUP)
	{
		serve_drink::BackupGoal backupGoal;
		acBackup.sendGoal(backupGoal);
		acBackup.waitForResult();
		serve_drink::BackupResultConstPtr backupResult = acBackup.getResult();

		if (backupResult->success == false)
		{
			state = STATE_NAVIGATION_2;
			ROS_INFO("Backup action failed");
			asServeDrinkResult.result_msg = "Backup from medicine counter failed, please complete the task manually.";
			asServeDrinkResult.success = false;
			asServeDrink.setSucceeded(asServeDrinkResult);
			return;
		}
		
		state = STATE_NAVIGATION_2;
	}
	
	if (state == STATE_NAVIGATION_2)
	{
		serve_drink::navigateGoal navGoal;
		navGoal.taskName = "Stage Front";
		navGoal.align = false;
		acNavigate.sendGoal(navGoal);
		acNavigate.waitForResult();
		serve_drink::navigateResultConstPtr navResult = acNavigate.getResult();

		if (navResult->success == false)
		{
			state = STATE_HANDOFF;
			ROS_INFO("Navigate to Couch action failed");
			asServeDrinkResult.result_msg = "Navigation to couch failed, please complete the task manually.";
			asServeDrinkResult.success = false;
			asServeDrink.setSucceeded(asServeDrinkResult);
			return;
		}
		
		state = STATE_HANDOFF;
	}
	
	if (state == STATE_HANDOFF)
	{
		/*
		serve_drink::handoffGoal handGoal;
		acHandoff.sendGoal(handGoal);
		acHandoff.waitForResult();
		serve_drink::handoffResultConstPtr handResult = acHandoff.getResult();

		if (handResult->success == false)
		{
			state = STATE_NAVIGATION_1;
			ROS_INFO("Medicine and Water Handoff action failed");
			asServeDrinkResult.result_msg = "Medicine and water delivery failed, please complete the task manually.";
			asServeDrinkResult.success = false;
			asServeDrink.setSucceeded(asServeDrinkResult);
			return;
		}
		
		//state = STATE_NAVIGATION_1;
		ROS_INFO("Retrieve Medicine action succeeded");
		asServeDrinkResult.result_msg = "Retrieve Medicine action succeeded";
		asServeDrinkResult.success = true;
		asServeDrink.setSucceeded(asServeDrinkResult);
		*/
		serve_drink::ReleaseGoal releaseGoal;
		acRelease.sendGoal(releaseGoal);
		acRelease.waitForResult();
		serve_drink::ReleaseResultConstPtr releaseResult = acRelease.getResult();

		if (releaseResult->success == false)
		{
			state = STATE_NAVIGATION_1;
			ROS_INFO("Medicine and Water Handoff action failed");
			asServeDrinkResult.result_msg = "Medicine and water delivery failed, please complete the task manually.";
			asServeDrinkResult.success = false;
			asServeDrink.setSucceeded(asServeDrinkResult);
			return;
		}
		
		state = STATE_NAVIGATION_1;
		ROS_INFO("Retrieve Medicine action succeeded");
		asServeDrinkResult.result_msg = "Retrieve Medicine action succeeded";
		asServeDrinkResult.success = true;
		asServeDrink.setSucceeded(asServeDrinkResult);
	}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serve_drink");

    serveDrink sd;

    ros::spin();

    return 0;
}


#include <retrieve_medicine/retrieve_medicine.h>

using namespace std;

retrieveMedicine::retrieveMedicine() : 
	acNavigate("/navigate_action", true),
	acHandoff("/handoff_action", true),
	acBackup("/backup_action", true),
	acPickupAll("/pickup_all_action", true),
	asRetrieveMedicine(n, "retrieve_medicine_action", boost::bind(&retrieveMedicine::executeRetrieveMedicine, this, _1), false)
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

	asRetrieveMedicine.start();
	
	state = STATE_NAVIGATION_1;
}

void retrieveMedicine::executeRetrieveMedicine(const retrieve_medicine::RetrieveMedicineGoalConstPtr& goal)
{
	//Navigate to medicine counter and align to counter
	if (state == STATE_NAVIGATION_1)
	{
		retrieve_medicine::navigateGoal navGoal;
		navGoal.taskName = "Medicine Nav";
		navGoal.align = true;
		acNavigate.sendGoal(navGoal);
		acNavigate.waitForResult();
		retrieve_medicine::navigateResultConstPtr navResult = acNavigate.getResult();

		if (navResult->success == false)
		{
			state = STATE_PICKUP;
			ROS_INFO("Navigate to Medicine Counter action failed");
			asRetrieveMedicineResult.result_msg = "Navigation to medicine counter failed, please complete the task manually.";
			asRetrieveMedicineResult.success = false;
			asRetrieveMedicine.setSucceeded(asRetrieveMedicineResult);
			return;
		}
		
		state = STATE_PICKUP;
	}
	
	//Pickup medicine and water from counter
	if (state == STATE_PICKUP)
	{
		retrieve_medicine::PickupAllGoal pickupGoal;
		acPickupAll.sendGoal(pickupGoal);
		acPickupAll.waitForResult();
		retrieve_medicine::PickupAllResultConstPtr pickupResult = acPickupAll.getResult();

		if (pickupResult->success == false)
		{
			state = STATE_BACKUP;
			ROS_INFO("Pickup Medicine and Water action failed");
			asRetrieveMedicineResult.result_msg = "Medicine and water pickup failed, please complete the task manually.";
			asRetrieveMedicineResult.success = false;
			asRetrieveMedicine.setSucceeded(asRetrieveMedicineResult);
			return;
		}
		
		state = STATE_BACKUP;
	}
	
	//Backup from medicine counter and tuck arms
	if (state == STATE_BACKUP)
	{
		retrieve_medicine::BackupGoal backupGoal;
		acBackup.sendGoal(backupGoal);
		acBackup.waitForResult();
		retrieve_medicine::BackupResultConstPtr backupResult = acBackup.getResult();

		if (backupResult->success == false)
		{
			state = STATE_NAVIGATION_2;
			ROS_INFO("Backup action failed");
			asRetrieveMedicineResult.result_msg = "Backup from medicine counter failed, please complete the task manually.";
			asRetrieveMedicineResult.success = false;
			asRetrieveMedicine.setSucceeded(asRetrieveMedicineResult);
			return;
		}
		
		state = STATE_NAVIGATION_2;
	}
	
	if (state == STATE_NAVIGATION_2)
	{
		retrieve_medicine::navigateGoal navGoal;
		navGoal.taskName = "Couch Dropoff";
		navGoal.align = false;
		acNavigate.sendGoal(navGoal);
		acNavigate.waitForResult();
		retrieve_medicine::navigateResultConstPtr navResult = acNavigate.getResult();

		if (navResult->success == false)
		{
			state = STATE_PICKUP;
			ROS_INFO("Navigate to Couch action failed");
			asRetrieveMedicineResult.result_msg = "Navigation to couch failed, please complete the task manually.";
			asRetrieveMedicineResult.success = false;
			asRetrieveMedicine.setSucceeded(asRetrieveMedicineResult);
			return;
		}
		
		state = STATE_HANDOFF;
	}
	
	if (state == STATE_HANDOFF)
	{
		retrieve_medicine::handoffGoal handGoal;
		acHandoff.sendGoal(handGoal);
		acHandoff.waitForResult();
		retrieve_medicine::handoffResultConstPtr handResult = acHandoff.getResult();

		if (handResult->success == false)
		{
			state = STATE_NAVIGATION_1;
			ROS_INFO("Medicine and Water Handoff action failed");
			asRetrieveMedicineResult.result_msg = "Medicine and water delivery failed, please complete the task manually.";
			asRetrieveMedicineResult.success = false;
			asRetrieveMedicine.setSucceeded(asRetrieveMedicineResult);
			return;
		}
		
		state = STATE_NAVIGATION_1;
		ROS_INFO("Retrieve Medicine action succeeded");
		asRetrieveMedicineResult.result_msg = "Retrieve Medicine action succeeded";
		asRetrieveMedicineResult.success = true;
		asRetrieveMedicine.setSucceeded(asRetrieveMedicineResult);
	}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "retrieve_medicine");

    retrieveMedicine rm;

    ros::spin();

    return 0;
}


#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <serve_drink_jun/navigateAction.h>
#include <serve_drink_jun/CallAction.h>

class SDClient
{
public:
  actionlib::SimpleActionClient<retrieve_medicine::navigateAction> ac;
  ros::ServiceServer navigateActionServer;
  ros::NodeHandle n;

  SDClient() : ac("navigate_action", true)
  {
    ROS_INFO("Waiting for server...");
    ac.waitForServer();
    ROS_INFO("Done");
    ROS_INFO("Advertising services...");
    navigateActionServer = n.advertiseService("call_action", &SDClient::callAction, this);

    ROS_INFO("Done");
  }
  
  
   bool callAction(retrieve_medicine::CallAction::Request &req, retrieve_medicine::CallAction::Response &res);    
};

bool SDClient::callAction(retrieve_medicine::CallAction::Request &req, retrieve_medicine::CallAction::Response &res)
{
   retrieve_medicine::navigateGoal goal;
   goal.taskName = req.taskName;
   goal.align = req.align;
   ac.sendGoal(goal);

   return true;   
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rm_client");

  SDClient rm;

  ros::spin();

  return 0;
}

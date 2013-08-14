#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <serve_drink/ServeDrinkAction.h>
#include <serve_drink/CallAction.h>

class sdClient
{
public:
  actionlib::SimpleActionClient<serve_drink::ServeDrinkAction> ac;
  ros::ServiceServer ServeDrinkActionServer;
  ros::NodeHandle n;

  sdClient() : ac("serve_drink_action", true)
  {
    ROS_INFO("Waiting for server...");
    ac.waitForServer();
    ROS_INFO("Done");
    ROS_INFO("Advertising services...");
    ServeDrinkActionServer = n.advertiseService("call_action", &sdClient::callAction, this);

    ROS_INFO("Done");
  }
  
  
   bool callAction(serve_drink::CallAction::Request &req, serve_drink::CallAction::Response &res);    
};

bool sdClient::callAction(serve_drink::CallAction::Request &req, serve_drink::CallAction::Response &res)
{
   serve_drink::ServeDrinkGoal goal;
   ac.sendGoal(goal);
   //goal.taskName = req.taskName;
   //goal.align = req.align;

   return true;   
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sd_client");

  sdClient sd;

  ros::spin();

  return 0;
}

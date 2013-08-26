#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <pr2_props/HighFiveAction.h>
#include <pr2_props/DoHighfive.h>

class HfClient
{
public:
  actionlib::SimpleActionClient<pr2_props::HighFiveAction> ac;
  ros::ServiceServer highFiveActionServer;
  ros::NodeHandle n;

  HfClient() : ac("highfive_action", true)
  {
    ROS_INFO("Waiting for server...");
    ac.waitForServer();
    ROS_INFO("Done");
    ROS_INFO("Advertising services...");
    highFiveActionServer = n.advertiseService("do_highfive", &HfClient::callAction, this);

    ROS_INFO("Done");
  }  
  
  bool callAction(pr2_props::DoHighfive::Request &req, pr2_props::DoHighfive::Response &res);    
};

bool HfClient::callAction(pr2_props::DoHighfive::Request &req, pr2_props::DoHighfive::Response &res)
{
   pr2_props::HighFiveGoal goal;
   ac.sendGoal(goal);
   //goal.taskName = req.taskName;
   //goal.align = req.align;

   return true;   
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hf_client");

  HfClient sd;

  ros::spin();

   return 0;
}

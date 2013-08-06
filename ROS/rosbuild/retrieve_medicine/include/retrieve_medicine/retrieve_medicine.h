#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <retrieve_medicine/navigateAction.h>

class retrieveMedicine
{
public: 
    ros::NodeHandle n;    

    actionlib::SimpleActionServer<retrieve_medicine::navigateAction> as;
    std::string actionName;
    retrieve_medicine::navigateFeedback asFeedback;
    retrieve_medicine::navigateResult asResult;

    retrieveMedicine(std::string name);

    void executeNavigate(const retrieve_medicine::navigateGoalConstPtr& goal);
};


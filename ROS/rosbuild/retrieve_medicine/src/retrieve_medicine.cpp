#include <retrieve_medicine/retrieve_medicine.h>

using namespace std;

retrieveMedicine::retrieveMedicine(string name) : as(n, name, boost::bind(&retrieveMedicine::executeNavigate, this, _1), false),
actionName(name)
{
    as.start();
}


void retrieveMedicine::executeNavigate(const retrieve_medicine::navigateGoalConstPtr& goal)
{
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "retrieve_medicine");

    retrieveMedicine rm("navigate_action");

    ros::spin();    

    return 0;
}


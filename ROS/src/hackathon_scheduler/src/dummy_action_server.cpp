#include <hackathon_scheduler/countAction.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<hackathon_scheduler::countAction> Server;

void execute(const hackathon_scheduler::countGoalConstPtr& goal, Server* as)
{
  ros::Rate rate(1);
  int count=0;
  hackathon_scheduler::countFeedback feedback;
  while (count < goal->count) {

    feedback.current=count;
    as->publishFeedback(feedback);

    count++;
    rate.sleep();
  }

  as->setSucceeded();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dummy_count_action_server");
  ros::NodeHandle n;
  Server server(n, "hackathon_scheduler/count", boost::bind(&execute, _1, &server), false);
  server.start();
  ros::spin();
  return 0;
}

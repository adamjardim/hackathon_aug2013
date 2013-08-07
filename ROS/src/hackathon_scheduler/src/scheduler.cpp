#include "ros/ros.h"
#include "hackathon_scheduler/Event.h"
#include "hackathon_scheduler/AddEvent.h"
#include "hackathon_scheduler/GetSchedule.h"
#include "hackathon_scheduler/RemoveEvent.h"
#include "hackathon_scheduler/TaskStatus.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"

#include <actionlib/client/simple_action_client.h>
#include <hackathon_scheduler/countAction.h>
#include <hackathon_scheduler/countResult.h>
#include <hackathon_scheduler/countFeedback.h>

//#include <something/lunchAction.h>
//#include <something/medicineAction.h>

#include <vector>
#include <string.h>
#include <time.h>

std::vector<hackathon_scheduler::Event> schedule;
ros::Publisher* taskStatusPublisher;
std::string taskName="";

//get the number of seconds since midnight from an hh:mm time string
long int secondsFromStringTime(std::string time) {
  long int hours,minutes;
  sscanf(time.c_str(),"%ld:%ld",&hours,&minutes);
  return hours*3600+minutes*60;
}

//get the current local time as an hh:mm time string
std::string getCurrentStringTime() {
  time_t rawtime;
  struct tm * timeinfo;

  time (&rawtime);
  timeinfo = localtime (&rawtime);
  char buf[5];
  sprintf(buf,"%2d:%2d",timeinfo->tm_hour,timeinfo->tm_min);
  return std::string(buf);
}

//print the whole schedule
void printSchedule() {
  ROS_INFO("Full schedule is now:");
  for (std::vector<hackathon_scheduler::Event>::iterator it = schedule.begin();
       it != schedule.end(); ++it)
  {
    ROS_INFO("    event %s of type %s with parameters {%s} at time %s",(*it).taskName.c_str(),(*it).taskType.c_str(),(*it).parameters.c_str(),(*it).startTime.c_str());
  } 
}

//struct for sorting events by start time
struct event_earlier_than_key
{
    inline bool operator() (const hackathon_scheduler::Event& struct1, const hackathon_scheduler::Event& struct2)
    {
        return (secondsFromStringTime(struct1.startTime) < secondsFromStringTime(struct2.startTime));
    }
};

//whether two events overlap
bool overlaps(hackathon_scheduler::Event e1, hackathon_scheduler::Event e2) {
  return (e1.startTime==e2.startTime);//!(e1.endTime<=e2.startTime || e2.endTime<=e1.startTime);
}

//add an Event to the schedule (and sort the schedule from earliest to latest)
bool addEvent(hackathon_scheduler::AddEvent::Request  &req,
         	 hackathon_scheduler::AddEvent::Response &res)
{
  hackathon_scheduler::Event e = req.event;
  ROS_INFO("Attempting to add event %s of type %s with parameters %s to schedule at time %s",e.taskName.c_str(),e.taskType.c_str(),e.parameters.c_str(),e.startTime.c_str());

  //determine if given event overlaps another event in the schedule
  for (std::vector<hackathon_scheduler::Event>::iterator it = schedule.begin();
       it != schedule.end(); ++it)
  {
    if (overlaps(e,*it)) {
      res.success=false;
      ROS_INFO("Events %s at %s of type %s and %s at %s of type %s overlap! Not adding event %s", 
          e.taskName.c_str(), e.startTime.c_str(), e.taskType.c_str(),
          (*it).taskName.c_str(), (*it).startTime.c_str(), (*it).taskType.c_str(),
          e.taskName.c_str());
      return true;
    }
  }

  //if no conflicts, add the event to the schedule
  schedule.push_back(e);
  ROS_INFO("Added event %s of type %s with parameters %s to schedule at time %s",e.taskName.c_str(),e.taskType.c_str(),e.parameters.c_str(),e.startTime.c_str());
  //sort the schedule from earliest to latest
  sort(schedule.begin(), schedule.end(), event_earlier_than_key());
  printSchedule();
  res.success=true;
  return true;
}

bool removeEvent(hackathon_scheduler::RemoveEvent::Request &req,
                 hackathon_scheduler::RemoveEvent::Response &res)
{
  ROS_INFO("Attempting to remove event at time %s",req.startTime.c_str());
  res.success=false;
  for (std::vector<hackathon_scheduler::Event>::iterator it = schedule.begin();
       it != schedule.end(); ++it)
  {
    if (secondsFromStringTime((*it).startTime)==secondsFromStringTime(req.startTime)) {
      res.success=true;
      ROS_INFO("Removing event %s of type %s at time %s from schedule",(*it).taskName.c_str(),(*it).taskType.c_str(),(*it).startTime.c_str());
      it=schedule.erase(it);
      break;
    }
  }
  return true;
}
//get the schedule as an array of events
bool getSchedule(hackathon_scheduler::GetSchedule::Request  &req,
         	 hackathon_scheduler::GetSchedule::Response &res)
{
  res.schedule.resize(schedule.size());
  int i=0;
  for (std::vector<hackathon_scheduler::Event>::iterator it = schedule.begin();
       it != schedule.end(); ++it)
  {
    res.schedule[i++]=(*it);
  }
  return true;
}


//test for adding events, adds a task1 called test with blank parameters at the current time+20 seconds
bool testAddEvent(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  ROS_INFO("Testing the addEvent service");
  hackathon_scheduler::Event e;
  e.taskName="dummy";
  e.startTime=getCurrentStringTime();//ros::Time::now()+ros::Duration(20);
  e.taskType="dummytask";
  e.parameters="5";
  hackathon_scheduler::AddEvent::Request r;
  r.event=e;
  hackathon_scheduler::AddEvent::Response o;
  ROS_INFO("Attempting to call service");
  if (!addEvent(r,o)) ROS_INFO("Problem");
  return true;
}

//test get the current time
bool testGetTime(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  ROS_INFO("Getting the current local time");
  std::string t = getCurrentStringTime();
  ROS_INFO("current time: %s, seconds since midnight: %ld",t.c_str(),secondsFromStringTime(t));
  return true;
}

bool testPublishStatus(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  ROS_INFO("Preparing to publish some test statuses");
  ros::NodeHandle n;
  ros::Rate rate(1);
  hackathon_scheduler::TaskStatus status;
  status.taskName="test";
  status.status="executing";
  for (int i=0; i<3; i++) {
    char buf[10];
    sprintf(buf,"%i",i);
    status.message=buf;
    taskStatusPublisher->publish(status);
    rate.sleep();
  }
  status.status="success";
  status.message="finished";
  taskStatusPublisher->publish(status);
  ros::spinOnce();
  rate.sleep();
  return true;
}

/////////Dummy Action callbacks
// Called once when the goal completes
void dummyActionDoneCb(const actionlib::SimpleClientGoalState& state,
            const hackathon_scheduler::countResultConstPtr& result)
{
  ROS_INFO("Dummy Action Finished");

  hackathon_scheduler::TaskStatus status;
  status.taskName=taskName;

  if (state==actionlib::SimpleClientGoalState::SUCCEEDED) {
    status.message=status.status="success";
    taskStatusPublisher->publish(status);
  }
  else {
    status.message=status.status="failure";
    taskStatusPublisher->publish(status);
  }
}

// Called once when the goal becomes active
void dummyActionActiveCb()
{
  ROS_INFO("Dummy action just went active");
  hackathon_scheduler::TaskStatus status;
  status.taskName=taskName;
  status.status="executing";
  status.message="starting dummy task";
  taskStatusPublisher->publish(status);
}

// Called every time feedback is received for the goal
void dummyActionFeedbackCb(const hackathon_scheduler::countFeedbackConstPtr& feedback)
{
  ROS_INFO("Got dummy action feedback: %d",feedback->current);
  hackathon_scheduler::TaskStatus status;
  status.taskName=taskName;
  status.status="executing";
  char buf[40];
  sprintf(buf,"dummy task count = %d",feedback->current);
  status.message=buf;
  taskStatusPublisher->publish(status);
}

void dummy_action(int count) {
  actionlib::SimpleActionClient<hackathon_scheduler::countAction> client("hackathon_scheduler/count", true); // true -> don't need ros::spin()
  client.waitForServer();
  hackathon_scheduler::countGoal goal;
  goal.count=count;
  client.sendGoal(goal, &dummyActionDoneCb,&dummyActionActiveCb,&dummyActionFeedbackCb);
  client.waitForResult();
}

bool testDummyAction(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  ROS_INFO("Beginning dummy action test");
  dummy_action(5);
  return true;
}
///////////////////

//////////medicine task
// Called once when the goal completes
/*void medicineActionDoneCb(const actionlib::SimpleClientGoalState& state,
            const something::medicineResultConstPtr& result)
{
  ROS_INFO("Dummy Action Finished");

  hackathon_scheduler::TaskStatus status;
  status.taskName=taskName;

  if (state==actionlib::SimpleClientGoalState::SUCCEEDED) {
    status.message=status.status="success";
    taskStatusPublisher->publish(status);
  }
  else {
    status.message=status.status="failure";
    taskStatusPublisher->publish(status);
  }
}

// Called once when the goal becomes active
void medicineActionActiveCb()
{
  ROS_INFO("Dummy action just went active");
  hackathon_scheduler::TaskStatus status;
  status.taskName=taskName;
  status.status="executing";
  status.message="starting medicine task";
  taskStatusPublisher->publish(status);
}

// Called every time feedback is received for the goal
void medicineActionFeedbackCb(const hackathon_scheduler::countFeedbackConstPtr& feedback)
{
  ROS_INFO("Got dummy action feedback: %d",feedback->current);
  hackathon_scheduler::TaskStatus status;
  status.taskName=taskName;
  status.status="executing";
  char buf[40];
  sprintf(buf,"dummy task count = %d",feedback->current);
  status.message=buf;
  taskStatusPublisher->publish(status);
}
*/
void get_medicine() {

 //run the medicine task
/*  actionlib::SimpleActionClient<something:medicineAction> client("something/medicine", true); // true -> don't need ros::spin()
  client.waitForServer();
  something::medicineGoal goal;
  client.sendGoal(goal, &medicineActionDoneCb,&medicineActionActiveCb,&medicineActionFeedbackCb);
  //wait until success or failure
  client.waitForResult();
  //see if medicine task failed and we need to do teleop
  if (client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) { //medicine task failed, should do teleop
    //if doing teleop, spin until you receive a resume medicine message/servicecall, then restart get_medicine
    should_restart_medicine=false;
    ros::Rate rate(5);
    while (!should_restart_medicine) {
      ros::spinOnce();
      rate.sleep();
    }
    get_medicine();
  }
*/
 //if success, finish

}
//////////
//////////lunch task
/*void lunchActionDoneCb(const actionlib::SimpleClientGoalState& state,
            const something::lunchResultConstPtr& result)
{
  ROS_INFO("Dummy Action Finished");

  hackathon_scheduler::TaskStatus status;
  status.taskName=taskName;

  if (state==actionlib::SimpleClientGoalState::SUCCEEDED) {
    status.message=status.status="success";
    taskStatusPublisher->publish(status);
  }
  else {
    status.message=status.status="failure";
    taskStatusPublisher->publish(status);
  }
}

// Called once when the goal becomes active
void lunchActionActiveCb()
{
  ROS_INFO("Dummy action just went active");
  hackathon_scheduler::TaskStatus status;
  status.taskName=taskName;
  status.status="executing";
  status.message="starting lunch task";
  taskStatusPublisher->publish(status);
}

// Called every time feedback is received for the goal
void medicineActionFeedbackCb(const hackathon_scheduler::countFeedbackConstPtr& feedback)
{
  ROS_INFO("Got dummy action feedback: %d",feedback->current);
  hackathon_scheduler::TaskStatus status;
  status.taskName=taskName;
  status.status="executing";
  char buf[40];
  sprintf(buf,"dummy task count = %d",feedback->current);
  status.message=buf;
  taskStatusPublisher->publish(status);
}
*/
void get_lunch(std::string template_name) {
  //get lunch with the given template
  /*actionlib::SimpleActionClient<something:lunchAction> client("something/lunch", true); // true -> don't need ros::spin()
  client.waitForServer();
  something::lunchGoal goal;
  goal.template=template_name;
  client.sendGoal(goal, &lunchActionDoneCb,&lunchActionActiveCb,&lunchActionFeedbackCb);
  //wait until success or failure
  client.waitForResult();
  */
}
//////////

void executeTask(hackathon_scheduler::Event task) {
  taskName=task.taskName;
  if (strstr(task.taskType.c_str(),"medicine")) {
    get_medicine();
  }
  else if (strstr(task.taskType.c_str(),"lunch")) {
    get_lunch(task.parameters);
  }
  else if (strstr(task.taskType.c_str(),"dummytask")) {
    int c;
    if (!sscanf(task.parameters.c_str(),"%d",&c)) c=5;
    dummy_action(c);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hackathon_scheduler");
  ros::NodeHandle n;

  ros::ServiceServer addEventService = n.advertiseService("hackathon_scheduler/addEvent", addEvent);
  //note: to remove an event from the command line, use '!!str hh:mm' for the time so it knows it's a string
  ros::ServiceServer removeEventService = n.advertiseService("hackathon_scheduler/removeEvent", removeEvent);
  ros::ServiceServer testService = n.advertiseService("hackathon_scheduler/testAddEvent",testAddEvent);
  ros::ServiceServer testGetTimeService = n.advertiseService("hackathon_scheduler/testGetTime",testGetTime);
  ros::ServiceServer getScheduleService = n.advertiseService("hackathon_scheduler/getSchedule",getSchedule);
  ros::ServiceServer testPublishStatusService = n.advertiseService("hackathon_scheduler/testPublishStatus",testPublishStatus);
  ros::ServiceServer testActionService = n.advertiseService("hackathon_scheduler/testActionService",testDummyAction);
  ros::Publisher p = n.advertise<hackathon_scheduler::TaskStatus>("hackathon_scheduler/status", 100);
  taskStatusPublisher = &p;
//  ros::Publisher taskStatusPublisher = n.advertise<hackathon_scheduler::TaskStatus>("hackathon_scheduler/status", 100);
  ROS_INFO("Add any schedule you want.");

  ros::Rate loop_rate(10.0); // 1Hz

  int check = 0;

  while (ros::ok())
  {
    loop_rate.sleep();
    ros::spinOnce();
    check++;
    //TODO: Replace this with something that checks all minutes since the last-checked minute
    //once a minute, check if a task is scheduled to start this minute
    if (check>=600) {
     ROS_INFO("A minute has passed. Checking for scheduled tasks at time %s",getCurrentStringTime().c_str());
     if (schedule.size()>0 && secondsFromStringTime(getCurrentStringTime()) == secondsFromStringTime(schedule.front().startTime)) {
       ROS_INFO("Found task %s of type %s! Attempting to execute",schedule.front().taskName.c_str(),schedule.front().taskType.c_str());
       executeTask(schedule.front());
     }
     check=0;
    }
  }

  return 0;
}

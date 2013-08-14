/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  \author Joe Romano
 *********************************************************************/
//@author  Joe Romano
//@email   joeromano@gmail.com
//@brief   low_five.cpp - pr2 gives you props yo

#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <pr2_gripper_sensor_msgs/PR2GripperEventDetectorAction.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>

typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;
// Our Action interface type, provided as a typedef for convenience                   
typedef actionlib::SimpleActionClient<pr2_gripper_sensor_msgs::PR2GripperEventDetectorAction> PlaceClient;
// Our Action interface type, provided as a typedef for convenience                   
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;
// Our Action interface type, provided as a typedef for convenience                   


class RobotArm
{
private:
  // Action client for the joint trajectory action 
  // used to trigger the arm movement action
  TrajClient* traj_client_r_;
  TrajClient* traj_client_l_;

public:
  //! Initialize the action client and wait for action server to come up
  RobotArm() 
  {
    // tell the action client that we want to spin a thread by default
    traj_client_r_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);
    traj_client_l_ = new TrajClient("l_arm_controller/joint_trajectory_action", true);

    // wait for action server to come up
    while(!traj_client_l_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the joint_trajectory_action server");
    }
    // wait for action server to come up
    while(!traj_client_r_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the joint_trajectory_action server");
    }
  }

  //! Clean up the action client
  ~RobotArm()
  {
    delete traj_client_r_;
    delete traj_client_l_;
  }

  //! Sends the command to start a given trajectory
  void startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal,bool right_arm)
  {
    //Start the trjaectory immediately
    goal.trajectory.header.stamp = ros::Time::now();

    if(right_arm)
      traj_client_r_->sendGoal(goal);
    else
      traj_client_l_->sendGoal(goal);
  }

  //! Generates a simple trajectory from a single waypoint
  pr2_controllers_msgs::JointTrajectoryGoal arm_trajectoryPoint(float* angles, float duration, bool right_arm)
  {
    //our goal variable
    pr2_controllers_msgs::JointTrajectoryGoal goal;

    // First, the joint names, which apply to all waypoints
    //starts at 17
    if(right_arm)
    {
      goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
      goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
      goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
      goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
      goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
      goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
      goal.trajectory.joint_names.push_back("r_wrist_roll_joint");
    }
    else
    {
      goal.trajectory.joint_names.push_back("l_shoulder_pan_joint");
      goal.trajectory.joint_names.push_back("l_shoulder_lift_joint");
      goal.trajectory.joint_names.push_back("l_upper_arm_roll_joint");
      goal.trajectory.joint_names.push_back("l_elbow_flex_joint");
      goal.trajectory.joint_names.push_back("l_forearm_roll_joint");
      goal.trajectory.joint_names.push_back("l_wrist_flex_joint");
      goal.trajectory.joint_names.push_back("l_wrist_roll_joint");
    }

    // We will have N waypoints in this goal trajectory
    goal.trajectory.points.resize(1);

    // First trajectory point
    // Positions
    int ind = 0;
    goal.trajectory.points[ind].positions.resize(7);
    goal.trajectory.points[ind].positions[0] =  angles[0];
    goal.trajectory.points[ind].positions[1] =  angles[1];
    goal.trajectory.points[ind].positions[2] =  angles[2];
    goal.trajectory.points[ind].positions[3] =  angles[3];
    goal.trajectory.points[ind].positions[4] =  angles[4];
    goal.trajectory.points[ind].positions[5] =  angles[5];
    goal.trajectory.points[ind].positions[6] =  angles[6];
    // Velocities
    goal.trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j)
    {
      goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // set time we want this trajectory to be reached at
    goal.trajectory.points[ind].time_from_start = ros::Duration(duration);

    //we are done; return the goal
    return goal;
  }


  //! Returns the current state of the action
  //actionlib::SimpleClientGoalState getState()
  //{
  //  return traj_client_->getState();
  //}
};


class Gripper{
private:
  GripperClient* gripper_client_r_, *gripper_client_l_;  
  PlaceClient* place_client_r_, *place_client_l_;

public:
  //Action client initialization
  Gripper(){

    //Initialize the client for the Action interface to the gripper controller
    //and tell the action client that we want to spin a thread by default
    gripper_client_l_  = new GripperClient("l_gripper_sensor_controller/gripper_action",true);
    place_client_l_  = new PlaceClient("l_gripper_sensor_controller/event_detector",true);
    gripper_client_r_  = new GripperClient("r_gripper_sensor_controller/gripper_action",true);
    place_client_r_  = new PlaceClient("r_gripper_sensor_controller/event_detector",true);
    
    //wait for the gripper action server to come up 
    while(!gripper_client_r_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the r_gripper_sensor_controller/gripper_action action server to come up");
    }

    while(!place_client_r_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the r_gripper_sensor_controller/event_detector action server to come up");
    }    

    //wait for the gripper action server to come up 
    while(!gripper_client_l_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the l_gripper_sensor_controller/gripper_action action server to come up");
    }

    while(!place_client_l_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the l_gripper_sensor_controller/event_detector action server to come up");
    }    


  }

  ~Gripper(){
    delete gripper_client_l_;
    delete gripper_client_r_;
    delete place_client_l_;
    delete place_client_r_;


  }

  


  //Open the gripper
  void open(bool left, bool right){
    pr2_controllers_msgs::Pr2GripperCommandGoal open;
    open.command.position = 0.09;    // position open (9 cm)
    open.command.max_effort = -1.0;  // Do not limit effort (negative)
    
    if(left)
      gripper_client_l_->sendGoal(open);
    if(right)
      gripper_client_r_->sendGoal(open);
  }

  //Close the gripper
  void close(bool left, bool right,bool wait_for_result){
    pr2_controllers_msgs::Pr2GripperCommandGoal close;
    close.command.position = 0.002;    // position open (9 cm)
    close.command.max_effort = -1.0;  // Do not limit effort (negative)
    
    if(left)
    {
      gripper_client_l_->sendGoal(close);
      if(wait_for_result)
	gripper_client_l_->waitForResult();
    }
    if(right)
    {
      gripper_client_r_->sendGoal(close);
      if(wait_for_result)
	gripper_client_r_->waitForResult();
    }
  }


  //move into place mode to drop an object
  void slap(bool left, bool right){
    pr2_gripper_sensor_msgs::PR2GripperEventDetectorGoal place_goal;
    place_goal.command.trigger_conditions = 4;  // just use acceleration as a trigger
    place_goal.command.acceleration_trigger_magnitude = 6.0;
    place_goal.command.slip_trigger_magnitude = 0.008;

    if(left)
      place_client_l_->sendGoal(place_goal);
    if(right)
      place_client_r_->sendGoal(place_goal);
  }

  //! Returns the current state of the action
  bool slapDone(bool left, bool right)
  {
    if(left && right)
      return place_client_l_->getState().isDone() && place_client_r_->getState().isDone();
    else if(left)
      return place_client_l_->getState().isDone();
    else if(right)
      return place_client_r_->getState().isDone();
    return false;
  }  
};

int main(int argc, char** argv){
  ros::init(argc, argv, "lift_test");

  // figure out what kind of five we want
  bool left = false;
  bool right = true;
  if(argc > 1)
  {
    left =  (strcmp(argv[1],"left") == 0);
    right = (strcmp(argv[1],"right") == 0);
  }



  RobotArm arm;
  Gripper gripper;

  gripper.close(left,right,true);
  
  float pre_gameday_r []= {-1.1036096831987545, 1.2273993160398446, 0.17540433749819506, -1.9961097303847053, -25.599756786728236, -0.64634984121557015, -22.883812446893003};
  float pre_gameday_l []= {1.4465628676193982, 1.2285750945849796, 0.40253450962996462, -2.0440306850200791, -31.168929834115303, -0.60889826357579968, 4.0359526956896818};
    if(left)
      arm.startTrajectory(arm.arm_trajectoryPoint(pre_gameday_l,2.0,false),false);
    if(right)
      arm.startTrajectory(arm.arm_trajectoryPoint(pre_gameday_r,2.0,true),true);
    sleep(2.0);

    float gameday_r []= {-0.12240986677882781, 0.98503486134558194, -0.37204703972859665, -1.2757306705402713, -25.884133066177196, -0.098050430120481424, -22.538873728343418};
    float gameday_l []= {0.1132586472843532, 0.84772875076872478, 0.5973656236693764, -1.1092462016166407, -31.294862942406674, -0.082788401731504702, 4.2950048059626678};
    if(left)
      arm.startTrajectory(arm.arm_trajectoryPoint(gameday_l,1.75,false),false);
    if(right)
      arm.startTrajectory(arm.arm_trajectoryPoint(gameday_r,1.75,true),true);

    // now start looking for a slap during the move
    gripper.slap(left,right);


    //wait for a slap
    while(!gripper.slapDone(left,right) && ros::ok())
    {
      ros::Duration(0.005).sleep();
    }


    if(left)
      arm.startTrajectory(arm.arm_trajectoryPoint(pre_gameday_l,1.35,false),false);
    if(right)
      arm.startTrajectory(arm.arm_trajectoryPoint(pre_gameday_r,1.35,true),true);
    //system("rosrun pr2_pose_saver load.py ./saved_poses/post_five.pps -s 1.0");


  return 0;
}

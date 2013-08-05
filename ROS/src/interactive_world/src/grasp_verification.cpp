#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <pr2_msgs/PressureState.h>
#include <interactive_world/GraspCheck.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>

#define FULLY_CLOSED .033
#define PRESSURE_THRESH 2800.0

class grasp_verification
{
public:

  grasp_verification()
  {
    l_avg = 0.0;
    r_avg = 0.0;

    l_gripper_pressure_sub = n.subscribe("/pressure/l_gripper_motor", 1, &grasp_verification::l_pressure_cb, this);
    r_gripper_pressure_sub = n.subscribe("/pressure/r_gripper_motor", 1, &grasp_verification::r_pressure_cb, this);

    grasp_verification_server = n.advertiseService("interactive_world/grasp_check", &grasp_verification::grasp_check,
                                                   this);

    r_grasp_publisher = n.advertise < std_msgs::Bool > ("/r_grasp_status", 1);
    l_grasp_publisher = n.advertise < std_msgs::Bool > ("/l_grasp_status", 1);
  }

  void publish_grasp_status()
  {
    //calculate gripper position using tfs
    tf::StampedTransform r_tf;
    tf::StampedTransform l_tf;
    try
    {
      tfl.lookupTransform("/l_gripper_r_finger_tip_link", "/l_gripper_l_finger_tip_link", ros::Time(0), l_tf);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
    }
    try
    {
      tfl.lookupTransform("/r_gripper_r_finger_tip_link", "/r_gripper_l_finger_tip_link", ros::Time(0), r_tf);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
    }
    double l_pos = sqrt(pow(l_tf.getOrigin().x(), 2) + pow(l_tf.getOrigin().y(), 2) + pow(l_tf.getOrigin().z(), 2));
    double r_pos = sqrt(pow(r_tf.getOrigin().x(), 2) + pow(r_tf.getOrigin().y(), 2) + pow(r_tf.getOrigin().z(), 2));

    std_msgs::Bool r_status;
    if (r_pos <= FULLY_CLOSED || r_avg <= PRESSURE_THRESH)
      r_status.data = false;
    else
      r_status.data = true;
    std_msgs::Bool l_status;
    if (l_pos <= FULLY_CLOSED || l_avg <= PRESSURE_THRESH)
      l_status.data = false;
    else
      l_status.data = true;
    r_grasp_publisher.publish(r_status);
    l_grasp_publisher.publish(l_status);
  }

  void r_pressure_cb(pr2_msgs::PressureState pressure)
  {
    std::vector < int16_t > l_finger = pressure.l_finger_tip;
    std::vector < int16_t > r_finger = pressure.r_finger_tip;

    double avg_pres = 0.0;
    for (unsigned int i = 0; i < l_finger.size(); i++)
    {
      avg_pres += l_finger.at(i);
    }
    for (unsigned int i = 0; i < r_finger.size(); i++)
    {
      avg_pres += r_finger.at(i);
    }
    avg_pres /= (float)(l_finger.size() + r_finger.size());
    r_avg = avg_pres;
  }

  void l_pressure_cb(pr2_msgs::PressureState pressure)
  {
    std::vector < int16_t > l_finger = pressure.l_finger_tip;
    std::vector < int16_t > r_finger = pressure.r_finger_tip;

    double avg_pres = 0.0;
    for (unsigned int i = 0; i < l_finger.size(); i++)
    {
      avg_pres += l_finger.at(i);
    }
    for (unsigned int i = 0; i < r_finger.size(); i++)
    {
      avg_pres += r_finger.at(i);
    }
    avg_pres /= (float)(l_finger.size() + r_finger.size());
    l_avg = avg_pres;
  }

  bool grasp_check(interactive_world::GraspCheck::Request &req, interactive_world::GraspCheck::Response &res)
  {
    //calculate gripper position using tfs
    tf::StampedTransform transform;
    double gripper_pressure;
    if (req.side.compare("left") == 0)
    {
      try
      {
        tfl.lookupTransform("/l_gripper_r_finger_tip_link", "/l_gripper_l_finger_tip_link", ros::Time(0), transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
      }
      gripper_pressure = l_avg;
    }
    else
    {
      try
      {
        tfl.lookupTransform("/r_gripper_r_finger_tip_link", "/r_gripper_l_finger_tip_link", ros::Time(0), transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
      }
      gripper_pressure = r_avg;
    }
    double gripper_pos = sqrt(
        pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2) + pow(transform.getOrigin().z(), 2));

    ROS_INFO("Position: %f, Pressure: %f", gripper_pos, gripper_pressure);
    if (gripper_pos <= FULLY_CLOSED || gripper_pressure <= PRESSURE_THRESH)
      res.isGrasping = false;
    else
      res.isGrasping = true;

    return true;
  }

private:
  ros::NodeHandle n;
  ros::ServiceServer grasp_verification_server;
  ros::Subscriber r_gripper_pressure_sub, l_gripper_pressure_sub;
  ros::Publisher r_grasp_publisher, l_grasp_publisher;
  tf::TransformListener tfl;
  double r_avg;
  double l_avg;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grasp_verification");

  grasp_verification gv;

  ros::Rate loopRate(30);
  while (ros::ok())
  {
    gv.publish_grasp_status();
    ros::spinOnce();
    loopRate.sleep();
  }

  return 0;
}

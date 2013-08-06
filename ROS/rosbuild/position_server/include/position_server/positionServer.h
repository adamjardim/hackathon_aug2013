#include <fstream>
#include <iostream>
#include <cstdlib>

#include <ros/ros.h>
#include <position_server/GetPosition.h>
#include <position_server/Position.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>

#include "yaml-cpp/yaml.h"

class positionServer
{
public:
	ros::NodeHandle n;
	ros::ServiceServer getPositionServer;
	
	std::map<std::string, position_server::Position> positions;
	
	positionServer();
	
	bool getPosition(position_server::GetPosition::Request &req, position_server::GetPosition::Response &res);
};


#include <position_server/positionServer.h>

using namespace std;

positionServer::positionServer() 
{
	//Read table positions from positions.yaml
	string filename;
	n.getParam("/position_server/position_filename", filename);
	ifstream fin(filename.c_str());
	YAML::Parser parser(fin);
	YAML::Node doc;
	parser.GetNextDocument(doc);
	for (unsigned i=0; i < doc.size(); i++)
	{
		position_server::Position pos;
		const YAML::Node& name = doc[i]["name"];
		const YAML::Node& position = doc[i]["position"];
		const YAML::Node& orientation = doc[i]["orientation"];
		const YAML::Node& height = doc[i]["height"];
		name >> pos.name;
		position[0] >> pos.pose.x;
		position[1] >> pos.pose.y;
		geometry_msgs::Quaternion q;
		orientation[0] >> q.x;
		orientation[1] >> q.y;
		orientation[2] >> q.z;
		orientation[3] >> q.w;
		height >> pos.height;
		pos.pose.theta = tf::getYaw(q);
		
		positions[pos.name] = pos;
	}
	
	getPositionServer = n.advertiseService("position_server/get_position", &positionServer::getPosition, this);
}

bool positionServer::getPosition(position_server::GetPosition::Request &req, position_server::GetPosition::Response &res)
{
	int count = positions.count(req.positionName);
	if (count > 0)
	{
		position_server::Position pos = positions[req.positionName];
		res.position = pos;
		return true;
	}
	else
		return false;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "position_server");
	
	positionServer ps;
	
	ros::spin();
	
	return 0;
}

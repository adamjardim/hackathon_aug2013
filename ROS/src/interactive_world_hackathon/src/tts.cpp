#include <ros/ros.h>
#include <interactive_world_hackathon/Speak.h>
#include <sstream>

class tts
{
public:

  tts()
  {
    serv = n.advertiseService("/tts/speak", &tts::speak, this);
    serv2 = n.advertiseService("/tts/play", &tts::play, this);
  }

  bool speak(interactive_world_hackathon::Speak::Request &req, interactive_world_hackathon::Speak::Response &res)
  {
    std::stringstream ss;
    ss << "pico2wave -w=/tmp/speak.wav \"" << req.text << "\"";
    system(ss.str().c_str());
    system("aplay /tmp/speak.wav");
    system("rm /tmp/speak.wav");
    return true;
  }

  bool play(interactive_world_hackathon::Speak::Request &req, interactive_world_hackathon::Speak::Response &res)
  {
    std::stringstream ss;
    ss << "aplay " << req.text;
    system(ss.str().c_str());
    return true;
  }

private:
  ros::NodeHandle n;
  ros::ServiceServer serv, serv2;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tts");
  tts t;
  ros::spin();
  return 0;
}

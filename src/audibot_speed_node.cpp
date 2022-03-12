#include <ros/ros.h>
#include "AudibotSpeedNode.hpp"

std::shared_ptr<audibot_twist_controller::AudibotSpeedNode> node;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "audibot_speed_control");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  node = std::make_shared<audibot_twist_controller::AudibotSpeedNode>(n, pn);
  
  ros::spin();
}
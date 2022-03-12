#include <ros/ros.h>
#include "AudibotSpeedNode.hpp"
#include "AudibotSteeringNode.hpp"

std::shared_ptr<audibot_twist_controller::AudibotSpeedNode> speed_node;
std::shared_ptr<audibot_twist_controller::AudibotSteeringNode> steering_node;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "audibot_twist_control");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  speed_node = std::make_shared<audibot_twist_controller::AudibotSpeedNode>(n, pn);
  steering_node = std::make_shared<audibot_twist_controller::AudibotSteeringNode>(n, pn);
  
  ros::spin();
}
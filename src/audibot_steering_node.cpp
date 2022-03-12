#include <ros/ros.h>
#include "AudibotSteeringNode.hpp"

std::shared_ptr<audibot_twist_controller::AudibotSteeringNode> node;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "audibot_steering_node");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  node = std::make_shared<audibot_twist_controller::AudibotSteeringNode>(n, pn);
  
  ros::spin();
}
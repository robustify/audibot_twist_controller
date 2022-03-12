#pragma once
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>

namespace audibot_twist_controller{

#define AUDIBOT_WHEELBASE         2.65
#define AUDIBOT_STEERING_RATIO    17.3

class AudibotSteeringNode
{
public:
  AudibotSteeringNode(ros::NodeHandle& n, ros::NodeHandle& pn);

private:
  void timerCallback(const ros::TimerEvent& event);
  void recvTwistCmd(const geometry_msgs::TwistConstPtr& msg);
  void recvTwistActual(const geometry_msgs::TwistStampedConstPtr& msg);

  ros::Subscriber sub_twist_;
  ros::Subscriber sub_twist_actual_;
  ros::Publisher pub_steering_cmd_;
  ros::Timer control_timer_;

  geometry_msgs::TwistStamped twist_cmd_;
  geometry_msgs::TwistStamped twist_actual_;

};

}

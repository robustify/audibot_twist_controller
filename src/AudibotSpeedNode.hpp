#pragma once
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>

#include <dynamic_reconfigure/server.h>
#include <audibot_twist_controller/AudibotSpeedConfig.h>

namespace audibot_twist_controller{
  
class AudibotSpeedNode
{
public:
  AudibotSpeedNode(ros::NodeHandle& n, ros::NodeHandle& pn);
  
private:
  void reconfig(AudibotSpeedConfig& config, uint32_t level);
  void timerCallback(const ros::TimerEvent& event);
  void recvTwistCmd(const geometry_msgs::TwistConstPtr& msg);
  void recvTwistActual(const geometry_msgs::TwistStampedConstPtr& msg);
  
  ros::Publisher pub_throttle_cmd_;
  ros::Publisher pub_brake_cmd;
  ros::Subscriber sub_twist_;
  ros::Subscriber sub_twist_actual_;
  ros::Timer control_timer_;
  
  dynamic_reconfigure::Server<AudibotSpeedConfig> srv_;
  AudibotSpeedConfig cfg_;
  
  geometry_msgs::TwistStamped twist_cmd_;
  geometry_msgs::TwistStamped twist_actual_;

  double int_throttle_;
};

}

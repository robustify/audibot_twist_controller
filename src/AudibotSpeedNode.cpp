#include "AudibotSpeedNode.hpp"

namespace audibot_twist_controller {
  
AudibotSpeedNode::AudibotSpeedNode(ros::NodeHandle& n, ros::NodeHandle& pn) :
  srv_(ros::NodeHandle(pn, "speed"))
{
  srv_.setCallback(boost::bind(&AudibotSpeedNode::reconfig, this, _1, _2));
  
  sub_twist_ = n.subscribe("cmd_vel", 1, &AudibotSpeedNode::recvTwistCmd, this);
  sub_twist_actual_ = n.subscribe("twist", 1, &AudibotSpeedNode::recvTwistActual, this);
  pub_throttle_cmd_ = n.advertise<std_msgs::Float64>("throttle_cmd", 1);
  pub_brake_cmd = n.advertise<std_msgs::Float64>("brake_cmd", 1);
  
  control_timer_ = n.createTimer(ros::Duration(0.02), &AudibotSpeedNode::timerCallback, this);
  
}

void AudibotSpeedNode::timerCallback(const ros::TimerEvent& event)
{
  if ((event.current_real - twist_cmd_.header.stamp).toSec() > 0.25){
    std_msgs::Float64 zero;
    zero.data = 0.0;
    pub_throttle_cmd_.publish(zero);
    return;
  }
  
  double speed_error = twist_cmd_.twist.linear.x - twist_actual_.twist.linear.x;
  int_throttle_ += 0.02 * cfg_.throttle_i * speed_error;
  if (int_throttle_ >= cfg_.max_throttle){
    int_throttle_ = cfg_.max_throttle;
  }else if (int_throttle_ < 0.0){
    int_throttle_ = 0.0;
  }
  
  std_msgs::Float64 throttle_cmd;
  throttle_cmd.data = cfg_.throttle_p * speed_error + cfg_.throttle_i * int_throttle_;
  if (throttle_cmd.data > cfg_.max_throttle){
    throttle_cmd.data = cfg_.max_throttle;
  }else if (throttle_cmd.data < 0.0){
    throttle_cmd.data = 0.0;
  }
  pub_throttle_cmd_.publish(throttle_cmd);
  
  std_msgs::Float64 brake_cmd;
  brake_cmd.data = -cfg_.brake_gain * speed_error;
  if (brake_cmd.data > cfg_.max_brake_force){
    brake_cmd.data = cfg_.max_brake_force;
  }else if (brake_cmd.data < 0.0){
    brake_cmd.data = 0.0;
  }
  pub_brake_cmd.publish(brake_cmd);
}

void AudibotSpeedNode::recvTwistCmd(const geometry_msgs::TwistConstPtr& msg)
{
  twist_cmd_.twist = *msg;
  twist_cmd_.header.stamp = ros::Time::now();
}

void AudibotSpeedNode::recvTwistActual(const geometry_msgs::TwistStampedConstPtr& msg)
{
  twist_actual_ = *msg;
}

void AudibotSpeedNode::reconfig(AudibotSpeedConfig& config, uint32_t level)
{
  cfg_ = config;
}
 
}
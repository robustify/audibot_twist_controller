#include "AudibotSteeringNode.hpp"

namespace audibot_twist_controller {
  
AudibotSteeringNode::AudibotSteeringNode(ros::NodeHandle& n, ros::NodeHandle& pn)
{
  sub_twist_ = n.subscribe("cmd_vel", 1, &AudibotSteeringNode::recvTwistCmd, this);
  sub_twist_actual_ = n.subscribe("twist", 1, &AudibotSteeringNode::recvTwistActual, this);
  pub_steering_cmd_ = n.advertise<std_msgs::Float64>("steering_cmd", 1);
  
  control_timer_ = n.createTimer(ros::Duration(0.02), &AudibotSteeringNode::timerCallback, this);
}
  
void AudibotSteeringNode::timerCallback(const ros::TimerEvent& event)
{
  if ((event.current_real - twist_cmd_.header.stamp).toSec() > 0.25){
    return;
  }
  
  std_msgs::Float64 steering_cmd;
  if (std::abs(twist_actual_.twist.linear.x) < 0.05){
    steering_cmd.data = 0.0;
  }else{
    steering_cmd.data = AUDIBOT_STEERING_RATIO * atan(AUDIBOT_WHEELBASE * twist_cmd_.twist.angular.z / twist_actual_.twist.linear.x);
  }
  
  pub_steering_cmd_.publish(steering_cmd);
}
  
void AudibotSteeringNode::recvTwistActual(const geometry_msgs::TwistStampedConstPtr& msg)
{
  twist_actual_ = *msg;
}

void AudibotSteeringNode::recvTwistCmd(const geometry_msgs::TwistConstPtr& msg)
{
  twist_cmd_.twist = *msg;
  twist_cmd_.header.stamp = ros::Time::now();
}
  
}

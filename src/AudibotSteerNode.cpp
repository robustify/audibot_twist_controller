#include "AudibotSteerNode.hpp"

namespace audibot_twist_controller {

AudibotSteerNode::AudibotSteerNode(const rclcpp::NodeOptions& options)
: rclcpp::Node("audibot_steer_node")
{
  sub_twist_ = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1, std::bind(&AudibotSteerNode::recvTwistCmd, this, std::placeholders::_1));
  sub_twist_actual_ = create_subscription<geometry_msgs::msg::TwistStamped>("twist", 1, std::bind(&AudibotSteerNode::recvTwistActual, this, std::placeholders::_1));
  pub_steering_cmd_ = create_publisher<example_interfaces::msg::Float64>("steering_cmd", 1);

  control_timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(20), std::bind(&AudibotSteerNode::timerCallback, this));
}

void AudibotSteerNode::timerCallback()
{
  rclcpp::Time twist_cmd_stamp(twist_cmd_.header.stamp);
  if (get_clock()->now().get_clock_type() != twist_cmd_stamp.get_clock_type()) {
    return;
  } else if ((get_clock()->now() - twist_cmd_stamp).seconds() > 0.25) {
    return;
  }

  example_interfaces::msg::Float64 steering_cmd;
  if (std::abs(twist_actual_.twist.linear.x) < 0.05){
    steering_cmd.data = 0.0;
  }else{
    steering_cmd.data = AUDIBOT_STEERING_RATIO * atan(AUDIBOT_WHEELBASE * twist_cmd_.twist.angular.z / twist_actual_.twist.linear.x);
  }

  pub_steering_cmd_->publish(steering_cmd);
}

void AudibotSteerNode::recvTwistActual(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  twist_actual_ = *msg;
}

void AudibotSteerNode::recvTwistCmd(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  twist_cmd_.twist = *msg;
  twist_cmd_.header.stamp = get_clock()->now();
}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(audibot_twist_controller::AudibotSteerNode)

#include "AudibotSpeedNode.hpp"

namespace audibot_twist_controller {

AudibotSpeedNode::AudibotSpeedNode(const rclcpp::NodeOptions& options)
: rclcpp::Node("audibot_speed_node")
{
  max_throttle = declare_parameter<double>("max_throttle", 1.0);
  max_brake_force = declare_parameter<double>("max_brake_force", 5000.0);
  throttle_p = declare_parameter<double>("throttle_p", 0.4);
  throttle_i = declare_parameter<double>("throttle_i", 0.1);
  brake_gain = declare_parameter<double>("brake_gain", 3000.0);

  sub_twist_ = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1, std::bind(&AudibotSpeedNode::recvTwistCmd, this, std::placeholders::_1));
  sub_twist_actual_ = create_subscription<geometry_msgs::msg::TwistStamped>("twist", 1, std::bind(&AudibotSpeedNode::recvTwistActual, this, std::placeholders::_1));
  pub_throttle_cmd_ = create_publisher<example_interfaces::msg::Float64>("throttle_cmd", 1);
  pub_brake_cmd_ = create_publisher<example_interfaces::msg::Float64>("brake_cmd", 1);

  control_timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(20), std::bind(&AudibotSpeedNode::timerCallback, this));
}

void AudibotSpeedNode::timerCallback()
{
  rclcpp::Time twist_cmd_stamp(twist_cmd_.header.stamp);
  if (get_clock()->now().get_clock_type() != twist_cmd_stamp.get_clock_type()) {
    example_interfaces::msg::Float64 zero;
    zero.data = 0.0;
    pub_throttle_cmd_->publish(zero);
    return;
  } else if ((get_clock()->now() - twist_cmd_stamp).seconds() > 0.25) {
    example_interfaces::msg::Float64 zero;
    zero.data = 0.0;
    pub_throttle_cmd_->publish(zero);
    return;
  }

  double speed_error = twist_cmd_.twist.linear.x - twist_actual_.twist.linear.x;
  int_throttle_ += 0.02 * throttle_i * speed_error;
  if (int_throttle_ >= max_throttle){
    int_throttle_ = max_throttle;
  }else if (int_throttle_ < 0.0){
    int_throttle_ = 0.0;
  }

  example_interfaces::msg::Float64 throttle_cmd;
  throttle_cmd.data = throttle_p * speed_error + throttle_i * int_throttle_;
  if (throttle_cmd.data > max_throttle){
    throttle_cmd.data = max_throttle;
  }else if (throttle_cmd.data < 0.0){
    throttle_cmd.data = 0.0;
  }
  pub_throttle_cmd_->publish(throttle_cmd);

  example_interfaces::msg::Float64 brake_cmd;
  brake_cmd.data = -brake_gain * speed_error;
  if (brake_cmd.data > max_brake_force){
    brake_cmd.data = max_brake_force;
  }else if (brake_cmd.data < 0.0){
    brake_cmd.data = 0.0;
  }
  pub_brake_cmd_->publish(brake_cmd);
}

void AudibotSpeedNode::recvTwistCmd(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  twist_cmd_.twist = *msg;
  twist_cmd_.header.stamp = get_clock()->now();
}

void AudibotSpeedNode::recvTwistActual(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  twist_actual_ = *msg;
}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(audibot_twist_controller::AudibotSpeedNode)

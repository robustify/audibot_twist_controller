#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <example_interfaces/msg/float64.hpp>

namespace audibot_twist_controller{

#define AUDIBOT_WHEELBASE         2.65
#define AUDIBOT_STEERING_RATIO    17.3

  class AudibotSteerNode : public rclcpp::Node {
    public:
      AudibotSteerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    private:
      void timerCallback();
      void recvTwistCmd(const geometry_msgs::msg::Twist::ConstSharedPtr msg);
      void recvTwistActual(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);

      rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
      rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_twist_actual_;
      rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr pub_steering_cmd_;
      rclcpp::TimerBase::SharedPtr control_timer_;

      geometry_msgs::msg::TwistStamped twist_cmd_;
      geometry_msgs::msg::TwistStamped twist_actual_;
  };
}

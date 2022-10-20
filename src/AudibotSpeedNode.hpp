#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <example_interfaces/msg/float64.hpp>

namespace audibot_twist_controller{

  class AudibotSpeedNode : public rclcpp::Node {
    public:
      AudibotSpeedNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    private:
      void timerCallback();
      void recvTwistCmd(const geometry_msgs::msg::Twist::ConstSharedPtr msg);
      void recvTwistActual(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);

      rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr pub_throttle_cmd_;
      rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr pub_brake_cmd_;
      rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
      rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_twist_actual_;
      rclcpp::TimerBase::SharedPtr control_timer_;

      geometry_msgs::msg::TwistStamped twist_cmd_;
      geometry_msgs::msg::TwistStamped twist_actual_;

      double int_throttle_;

      // ROS Parameters
      double max_throttle;
      double max_brake_force;
      double throttle_p;
      double throttle_i;
      double brake_gain;
  };
}

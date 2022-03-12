# Audibot Twist Controller

This repository contains nodes to implement closed-loop speed control and kinematic steering control for the Audibot Gazebo simulator:

[https://github.com/robustify/audibot](https://github.com/robustify/audibot).

These three nodes all subscribe to a `geometry_msgs/Twist` topic:
- `audibot_speed_node` sends brake and throttle commands to Audibot to track an input vehicle speed command.
- `audibot_steering_node` sends steering angle commands to Audibot to track an input yaw rate command.
- `audibot_twist_node` combines both of the above nodes into one

The `linear.x` field of the command is used as the target vehicle speed which the speed control component uses to output brake and throttle commands.

The `angular.z` field is used as the target yaw rate the steering control component uses to output steering wheel angle commands.

An example launch file called `example.launch` brings up the simulator and the `audibot_twist_node`.
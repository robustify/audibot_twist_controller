# Audibot Twist Controller

This repository contains nodes to implement closed-loop speed control and kinematic steering control for the Audibot Gazebo simulator:

[https://github.com/robustify/audibot/tree/ros2](https://github.com/robustify/audibot/tree/ros2).

Each node subscribes to a `geometry_msgs/msg/Twist` command topic. Then:
- `audibot_speed_node` sends brake and throttle commands to Audibot to track a vehicle speed command.
- `audibot_steer_node` sends steering angle commands to Audibot to track a yaw rate command.

The `linear.x` field of the command is used as the target vehicle speed which the speed control component uses to output brake and throttle commands.

The `angular.z` field is used as the target yaw rate the steering control component uses to output steering wheel angle commands.

An example launch file called `example.launch.xml` brings up the simulator, `audibot_speed_node`, and `audibot_steer_node`.
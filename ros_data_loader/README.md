# ROS Data Loader
This library is used to load parameters into ROS parameter server.

Moveit has provided convenient interfaces to load robot description and
configuration from ROS parameter server. But in general, these data is loaded
through launch file, which is not programmable.

This library provides a helper that can manage data required by moveit through
function call directly.
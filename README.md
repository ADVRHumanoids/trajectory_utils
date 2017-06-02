trajectory_utils
================
The purpose of this library is to have a wrapper around the KDL::Trajectories in order to simplify the usage and the interface.
The repo contains also the ROS node ```trajectory_designed```.

trajectory_designer
-------------------
```trajectory_designed``` is a ROS node which aim is to provide a simple tool to fast sketch Cartesian trajectories using ```interactive_markers``` in RVIZ:
[![trajectory_designer](https://img.youtube.com/vi/HPBwuupm1Fo/0.jpg)](https://www.youtube.com/watch?v=HPBwuupm1Fo)

The Cartesian trajectories available at the moment are:
- Minimum Jerk
- Semicircular in along the local planes XY, YZ and XZ

### Subscribed Topic:
```_distal_link_goal```([geometry_msgs::Pose](http://docs.ros.org/api/geometry_msgs/html/msg/Pose.html)): Set a goal from outside

### Published Topic:


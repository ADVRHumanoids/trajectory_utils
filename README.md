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

#### Subscribed Topics:
```distal_link_goal```([geometry_msgs::Pose](http://docs.ros.org/api/geometry_msgs/html/msg/Pose.html)): set a goal from outside

#### Published Topics:
```distal_link_segments```(trajectory_utils/segmentTrj): store the data of each trajectory segment

```distal_link_trj```(trajectory_utils/CartesianTrj): store the complete trajectory

```distal_link_trj_viz```([nav_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html)): used for visualization purposes

#### Services:
```distal_link_getTrj```([std_srvs/Empty](http://docs.ros.org/api/std_srvs/html/srv/Empty.html)): request publication of actual trajectory and segments

#### Parameters:
```~base_link```(```string```): frame in which the ```distal_frame``` is controlled

```~distal_link```(```string```): controlled frame

```~dT```(```double```): trajectory sample

#### Required tf Transforms:
```\base_link``` -> ```\distal_link``` usually given by the [```robot_state_publisher```](http://wiki.ros.org/robot_state_publisher) node

#### Messages:
[```Cartesian```](https://github.com/ADVRHumanoids/trajectory_utils/blob/master/msg/Cartesian.msg)

[```CartesianTrj```](https://github.com/ADVRHumanoids/trajectory_utils/blob/master/msg/CartesianTrj.msg)

[```segment```](https://github.com/ADVRHumanoids/trajectory_utils/blob/master/msg/segment.msg)

[```segmentTrj```](https://github.com/ADVRHumanoids/trajectory_utils/blob/master/msg/segmentTrj.msg)

#### Marker Menu Entries:
- ```MinJerk```: creates a linear minimum jerk trajectory between the last pose and the actual pose of the marker of time T
- ```SemiCircular```: creates a semicircular trajectory between the last pose and the actual pose of the marker of time T. Note that the actual pose of the marker has to lie on the local XY, YZ or ZX plane, if not the trajectory will not be created. The ```Reverse``` entry, if selected, change the path between the two possible semicircles
- ```Reset Marker Pose```: sets the actual pose of the marker to the last one
- ```Reset Trajectory```: permits to reset the ```Last``` added trajectory or ```All``` the added trajectories
- ```Goal```: permits to move to the set external goal using a minimum jerk trajectory of time T (with the ```Move to Goal in T [sec]``` entry), to remove the goal (with the ```Remove Goal``` entry) or to set the actual pose of the marker (with the ```Set Marker Pose``` entry)
- ```Restart Marker```: will reset all the stored trajectories and the marker pose to the actual pose of the ```distal_link``` (from ```tf```)

# OpenUGV ROS2 packages and information

## Application of Simple Commander API in waypoints following
This package demonstrates the use of the Simple Commander API in waypoints following for the Husarion Panther robot in gazebo simulation environment. However, this code can also be used for the real panther robot with some modfications to the location or position of poses.


### commander_api package
The package contains two executable files. In one of the files are given poses for the Rosbot-xl robot while in the other are poses for the Panther. The idea is for the robot to follow the waypoint to each of the poses in turn until the last pose.

To use this repository (commander_api) package, one will need to clone this package and build it locally with the command:

```bash
colcon build --packages-select commander_api
```

or just <br>

```bash
colcon build
```

if there aren't many packages already in the workspace.
# OpenUGV ROS2 packages and information

## Application of Simple Commander API in waypoints following
This package demonstrates the use of the Simple Commander API in waypoints following for the Husarion Panther robot in gazebo simulation environment. However, this code can also be used for the real panther robot with some modfications to the location or position of poses.


### commander_api and py_srvcli packages
The package contains many executable files. In one of the files are given poses for the Rosbot-xl robot while in the other are poses for the Panther. The idea is for the robot to follow the waypoint to each of the poses in turn until the last pose. The updated information about this repository is given below under the topic `New updates`

To use this repository (commander_api) package, one will need to clone this package and build it locally with the command:

```bash
colcon build --packages-select commander_api
```

or just <br>

```bash
colcon build
```

if there aren't many packages already in the workspace.

### New updates

There has been further modifications to this package with the additions of more functionalities. For instance, it is now possible to use the coordinates saved in a csv file for waypoint following. The .csv file is saved in the `"/install/commander_api/share/commander_api/config/"` directory of the workspace. 

The file is generated when the client_member_function.py client node from the "py_srvcli" package is executed. This needs the server (service_member_function.py) node to be running. The server generates the coordinates and send it as a response to the client node. Each time the client is executed, it requests for the present coordinate of the robot and append it to the csv file in the above directory. To not make things complicated, it is recommended to generate a new csv file for each waypoints following tasks.

Previous files can be copied into some other locations for reference if need be, since the application is made in a way that it appends a new coordinates received to the contents of the file.

If the robot navigation stack is already started, the application works as follows:

> Run the following commands in the order shown.
> - `ros2 run py_srvcli service`
> - `ros2 run py_srvcli client map`

The first command above starts the server while the second command starts the client which makes a request to the server for the current coordinates of the robot in the map frame. After receiving the coordinates, the client node shutdown.

After running the client for as many times as we want, we can then proceed by running the waypoint following script. This script makes use of the coordinates in the generated csv file and executes waypoint following to each of the coordinates one after the other until the last one.

```bash
ros2 run commander_api real_panther_waypoint_following_with_class_definition
```

## Note:
Running the `real_panther_waypoint_following_with_class_definition.py` file from the commander_api package will likely throw an exception if the `pyttsx3` (text to voice/speech) library is not installed. So, it is enough to comment out line 6, 72, 75 and the `text_to_speech` method.
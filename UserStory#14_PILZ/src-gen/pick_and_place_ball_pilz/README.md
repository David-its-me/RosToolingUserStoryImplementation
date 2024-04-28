# pick_and_place_ball_PILZ

This package has be created automatically using the [RosTooling](https://github.com/ipa320/RosTooling).


It holds the launch file to run the following nodes:
- moveit_config_server
- bt_operator
- moveit_skill_server
- gripper_command_action_server
- lifecycle_manager

The listed nodes offer the following connections:
- ActionServer: start_application [man2_msgs/RunApplication]

## Installation

### Using release

This package can be copied to a valid ROS 2 workspace. To be sure that all the related dependencies are intalles the command **rosdep install** can be used.
Then the workspace must be compiled using the common ROS 2 build command:

```
mkdir -p ros2_ws/src
cd ros2_ws/
cp PATHtoTHISPackage/pick_and_place_ball_PILZ src/. 
rosdep install --from-path src/ -i -y
colcon build
source install/setup.bash
```



## Usage


To execute the launch file, the following command can be called:

```
ros2 launch pick_and_place_ball_PILZ pick_and_place_ball_PILZ.launch.py 
```

The generated launch files requires the xterm package, it can be installed by:

```
sudo apt install xterm
```




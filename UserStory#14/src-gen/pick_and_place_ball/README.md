# pick_and_place_ball

This package has be created automatically using the [RosTooling](https://github.com/ipa320/RosTooling).


It holds the launch file to run the following nodes:
- bt_operator
- moveit_skill_server
- ur_io_control_gripper_skill_server
- lifecycle_manager

The listed nodes offer the following connections:
- ActionServer: start_application [man2_msgs/RunApplication]
- ServiceServer: get_state_bt_operator_server [lifecycle_msgs/GetState]
- ServiceServer: change_state_bt_operator_server [lifecycle_msgs/ChangeState]
- ServiceClient: get_state_aruco_marker_action_server_client [lifecycle_msgs/GetState]
- ServiceClient: change_state_arcuro_marker_action_server_client [lifecycle_msgs/ChangeState]
- ServiceClient: get_state_bt_operator_client [lifecycle_msgs/GetState]
- ServiceClient: change_state_bt_operator_client [lifecycle_msgs/ChangeState]

## Installation

### Using release

This package can be copied to a valid ROS 2 workspace. To be sure that all the related dependencies are intalles the command **rosdep install** can be used.
Then the workspace must be compiled using the common ROS 2 build command:

```
mkdir -p ros2_ws/src
cd ros2_ws/
cp PATHtoTHISPackage/pick_and_place_ball src/. 
rosdep install --from-path src/ -i -y
colcon build
source install/setup.bash
```



## Usage


To execute the launch file, the following command can be called:

```
ros2 launch pick_and_place_ball pick_and_place_ball.launch.py
```

The generated launch files requires the xterm package, it can be installed by:

```
sudo apt install xterm
```




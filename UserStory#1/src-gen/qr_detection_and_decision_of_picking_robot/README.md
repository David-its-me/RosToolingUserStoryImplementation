# qr_detection_and_decision_of_picking_robot

This package has be created automatically using the [RosTooling](https://github.com/ipa320/RosTooling).


It holds the launch file to run the following nodes:
- marker_publisher
- realsense_tf_node
- realsense_camera_driver
- bt_operator
- detect_acuco_marker_action_server
- moveit_skill_server
- ur_io_control_gripper_skill_server
- lifecycle_manager

The listed nodes offer the following connections:
- Subscriber: camera_info_sub [sensor_msgs/CameraInfo]
- Subscriber: camera_image_sub [sensor_msgs/Image]
- Publisher: /camera_info [sensor_msgs/CameraInfo]
- Publisher: /camera_image [sensor_msgs/Image]
- ActionServer: start_application [man2_msgs/RunApplication]
- ServiceServer: get_state_bt_operator_server [lifecycle_msgs/GetState]
- ServiceServer: change_state_bt_operator_server [lifecycle_msgs/ChangeState]
- ServiceServer: get_state_aruco_marker_action_server [lifecycle_msgs/GetState]
- ServiceServer: change_state_aruco_marker_action_server [lifecycle_msgs/ChangeState]
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
cp PATHtoTHISPackage/qr_detection_and_decision_of_picking_robot src/. 
rosdep install --from-path src/ -i -y
colcon build
source install/setup.bash
```


### From source code
```
mkdir -p ros2_ws/src
cd ros2_ws/
git clone https://github.com/pal-robotics/aruco_rosgit clone https://github.com/ipa-rwu/realsense_camera_calibrated.gitgit clone https://gitlab.cc-asp.fraunhofer.de/ipa326/demonstrator/bt_based_application_framework.git -b 50y_demogit clone https://gitlab.cc-asp.fraunhofer.de/ipa326/demonstrator/bt_based_application_frameworkgit clone https://gitlab.cc-asp.fraunhofer.de/ipa326/demonstrator/bt_based_application_frameworkgit clone https://gitlab.cc-asp.fraunhofer.de/ipa326/demonstrator/bt_based_application_frameworkgit clone https://github.com/ros-planning/navigation2/tree/main
rosdep install --from-path src/ -i -y
colcon build
source install/setup.bash
```

## Usage


To execute the launch file, the following command can be called:

```
ros2 launch qr_detection_and_decision_of_picking_robot qr_detection_and_decision_of_picking_robot.launch.py
```

The generated launch files requires the xterm package, it can be installed by:

```
sudo apt install xterm
```




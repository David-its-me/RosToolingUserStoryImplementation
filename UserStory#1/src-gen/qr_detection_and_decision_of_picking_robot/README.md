# qr_detection_and_decision_of_picking_robot

This package has be created automatically using the [RosTooling](https://github.com/ipa320/RosTooling).


It holds the launch file to run the following nodes:
- marker_publisher
- realsense_tf_node
- realsense_camera_driver
- moveit_config_server
- bt_operator
- detect_acuco_marker_action_server
- moveit_skill_server
- ur_io_control_gripper_skill_server
- lifecycle_manager

The listed nodes offer the following connections:
- Subscriber: camera_info_sub [sensor_msgs/CameraInfo]
- Subscriber: camera_image_sub [sensor_msgs/Image]
- Publisher: /camera/realsense_camera_driver/color/camera_info [sensor_msgs/CameraInfo]
- Publisher: /camera/realsense_camera_driver/color/image_raw [sensor_msgs/Image]
- ActionServer: start_application [man2_msgs/RunApplication]

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



## Usage


To execute the launch file, the following command can be called:

```
ros2 launch qr_detection_and_decision_of_picking_robot qr_detection_and_decision_of_picking_robot.launch.py
```

The generated launch files requires the xterm package, it can be installed by:

```
sudo apt install xterm
```




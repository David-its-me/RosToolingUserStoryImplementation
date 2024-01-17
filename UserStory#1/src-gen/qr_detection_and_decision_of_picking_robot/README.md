# qr_detection_and_decision_of_picking_robot

This package has be created automatically using the [RosTooling](https://github.com/ipa320/RosTooling).


It holds the launch file to run the following nodes:
- realsense_driver
- aruco_ros

The listed nodes offer the following connections:
- Publisher: /camera_info [sensor_msgs/CameraInfo]
- Publisher: /camera_image [sensor_msgs/Image]
- Subscriber: camera_info_sub [sensor_msgs/CameraInfo]
- Subscriber: camera_image_sub [sensor_msgs/Image]

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




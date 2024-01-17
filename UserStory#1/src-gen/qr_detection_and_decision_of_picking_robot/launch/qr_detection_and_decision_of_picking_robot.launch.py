from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
def generate_launch_description():
    ld = LaunchDescription()

    realsense_driver = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        prefix = 'xterm -e',
        output='screen',
        name="realsense_driver",
        remappings=[
          ("/camera/realsense_driver/color/camera_info", "/camera_info"),
          ("/camera/realsense_driver/color/image_raw", "/camera_image")]
    )

    aruco_ros = Node(
        package="aruco_ros",
        executable="marker_publisher",
        prefix = 'xterm -e',
        output='screen',
        name="aruco_ros",
        remappings=[
          ("camera_info", "/camera_info"),
          ("image", "/camera_image")]
    )

    
    include_ur5e_cell_moveit_config= IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('ur5e_cell_moveit_config') + '/launch/robot.launch.py'])
    )

    ld.add_action(realsense_driver)
    ld.add_action(aruco_ros)
    ld.add_action(include_ur5e_cell_moveit_config)

    return ld

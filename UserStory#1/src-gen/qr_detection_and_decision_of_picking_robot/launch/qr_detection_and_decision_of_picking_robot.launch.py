from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
def generate_launch_description():
    ld = LaunchDescription()
    
    frame_arg = DeclareLaunchArgument(
        "frame", default_value=TextSubstitution(text="world")
    )
    behaviour_tree_xml_path_arg = DeclareLaunchArgument(
        "behaviour_tree_xml_path", default_value=TextSubstitution(text="test_bt/SimpleMovement.xml")
    )
    behaviour_tree_name_arg = DeclareLaunchArgument(
        "behaviour_tree_name", default_value=TextSubstitution(text="MoveToQR")
    )

    realsense_camera_calibrated = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        prefix = 'xterm -e',
        output='screen',
        name="realsense_camera_calibrated",
        remappings=[
          ("/camera/realsense_driver/color/camera_info", "/camera_info"),
          ("/camera/realsense_driver/color/image_raw", "/camera_image")]
    )
    aruco_marker_publisher = Node(
        package="aruco_ros",
        executable="marker_publisher",
        prefix = 'xterm -e',
        output='screen',
        name="aruco_marker_publisher",
        remappings=[
          ("camera_info", "/camera_info"),
          ("image", "/camera_image")]
        ,
        parameters=[{
        "reference_frame": LaunchConfiguration("frame"),}]
    )
    detect_acuco_marker_action_server = Node(
        package="detect_aruco_marker_skill",
        executable="detect_aruco_marker_action_server",
        prefix = 'xterm -e',
        output='screen',
        name="detect_acuco_marker_action_server"
    )
    moveit_skill_server = Node(
        package="moveit_skills",
        executable="moveit_skill_server",
        prefix = 'xterm -e',
        output='screen',
        name="moveit_skill_server"
    )
    ur_io_control_gripper_skill_server = Node(
        package="io_control_gripper_skill",
        executable="ur_io_control_gripper_action_server",
        prefix = 'xterm -e',
        output='screen',
        name="ur_io_control_gripper_skill_server"
    )
    bt_operator = Node(
        package="man2_bt_operator",
        executable="bt_operator",
        prefix = 'xterm -e',
        output='screen',
        name="bt_operator",
        parameters=[{
        "current_bt_xml_filename": LaunchConfiguration("behaviour_tree_xml_path"),
        "current_bt_xml_name": LaunchConfiguration("behaviour_tree_name"),}]
    )
    include_ur5e_cell_moveit_config= IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('ur5e_cell_moveit_config') + '/launch/robot.launch.py'])
    )

    ld.add_action(realsense_camera_calibrated)
    ld.add_action(aruco_marker_publisher)
    ld.add_action(detect_acuco_marker_action_server)
    ld.add_action(moveit_skill_server)
    ld.add_action(ur_io_control_gripper_skill_server)
    ld.add_action(bt_operator)
    ld.add_action(include_ur5e_cell_moveit_config)

    return ld

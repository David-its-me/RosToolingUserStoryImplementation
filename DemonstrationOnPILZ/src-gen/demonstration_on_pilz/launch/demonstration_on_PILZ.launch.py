from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution, TextSubstitution

def generate_launch_description():
    ld = LaunchDescription()

    camera_frame_arg = DeclareLaunchArgument(
        "camera_frame", default_value=TextSubstitution(text="camera_color_optical_frame")
    )
    ld.add_action(camera_frame_arg)
    reference_frame_arg = DeclareLaunchArgument(
        "reference_frame", default_value=TextSubstitution(text="tool0")
    )
    ld.add_action(reference_frame_arg)
    marker_size_arg_arg = DeclareLaunchArgument(
        "marker_size_arg", default_value=TextSubstitution(text="0.05")
    )
    ld.add_action(marker_size_arg_arg)
    raw_image_topic_arg = DeclareLaunchArgument(
        "raw_image_topic", default_value=TextSubstitution(text="camera/color/image_raw")
    )
    ld.add_action(raw_image_topic_arg)
    camera_info_topic_arg = DeclareLaunchArgument(
        "camera_info_topic", default_value=TextSubstitution(text="camera/color/camera_info")
    )
    ld.add_action(camera_info_topic_arg)
    realsense_tf_node_config = os.path.join(
        get_package_share_directory('demonstration_on_pilz'),
        'config',
        'realsense_tf_node.yaml'
        )
    camera_name_arg = DeclareLaunchArgument(
        "camera_name", default_value=TextSubstitution(text="camera")
    )
    ld.add_action(camera_name_arg)
    device_type_arg = DeclareLaunchArgument(
        "device_type", default_value=TextSubstitution(text="d435")
    )
    ld.add_action(device_type_arg)
    publish_tf_arg = DeclareLaunchArgument(
        "publish_tf", default_value=TextSubstitution(text="false")
    )
    ld.add_action(publish_tf_arg)
    frame_id_arg = DeclareLaunchArgument(
        "frame_id", default_value=TextSubstitution(text="tool0")
    )
    ld.add_action(frame_id_arg)
    child_frame_link_arg = DeclareLaunchArgument(
        "child_frame_link", default_value=TextSubstitution(text="camera_link")
    )
    ld.add_action(child_frame_link_arg)
    robot_name_arg = DeclareLaunchArgument(
        "robot_name", default_value=TextSubstitution(text="prbt_cell")
    )
    ld.add_action(robot_name_arg)
    moveit_config_arg = DeclareLaunchArgument(
        "moveit_config", default_value=TextSubstitution(text="prbt_cell_moveit_config")
    )
    ld.add_action(moveit_config_arg)
    current_bt_xml_filename_arg = DeclareLaunchArgument(
        "current_bt_xml_filename", default_value=TextSubstitution(text="Masterarbeit_Cloud/ma-lieb-robotics-integration/04_Umsetzung/Implementation/Behaviour Tree/SimpleMovement.xml -p run_loop:=false")
    )
    ld.add_action(current_bt_xml_filename_arg)
    current_bt_xml_name_arg = DeclareLaunchArgument(
        "current_bt_xml_name", default_value=TextSubstitution(text="MoveToQR")
    )
    ld.add_action(current_bt_xml_name_arg)
    man2_bt_skill_clients_arg = DeclareLaunchArgument(
        "man2_bt_skill_clients", default_value=TextSubstitution(text="[compute_path_to_state_client, compute_path_to_pose_client, compute_path_to_point_client, execute_trajectory_client, set_path_constraints, get_current_ik_frame_pose_client, update_pose, split_pose, create_pose, update_orientation_in_pose, detect_aruco_marker_client, print_value, print_ros_msg]")
    )
    ld.add_action(man2_bt_skill_clients_arg)
    ur_robot_skill_clients_arg = DeclareLaunchArgument(
        "ur_robot_skill_clients", default_value=TextSubstitution(text="[io_control_gripper, get_ur_robot_state, get_ur_safety_state, get_ur_program_state, check_safety_mode, check_robot_mode, is_robot_running, is_safety_normal, is_program_running]")
    )
    ld.add_action(ur_robot_skill_clients_arg)
    default_plugin_lib_names_arg = DeclareLaunchArgument(
        "default_plugin_lib_names", default_value=TextSubstitution(text="[util_plugin_print_value, rest_action_observer]")
    )
    ld.add_action(default_plugin_lib_names_arg)
    detect_acuco_marker_action_server_config = os.path.join(
        get_package_share_directory('demonstration_on_pilz'),
        'config',
        'detect_acuco_marker_action_server.yaml'
        )
    moveit_skill_server_config = os.path.join(
        get_package_share_directory('demonstration_on_pilz'),
        'config',
        'moveit_skill_server.yaml'
        )
    autostart_arg = DeclareLaunchArgument(
        "autostart", default_value=TextSubstitution(text="true")
    )
    ld.add_action(autostart_arg)
    node_names_arg = DeclareLaunchArgument(
        "node_names", default_value=TextSubstitution(text="[moveit_config_server, bt_operator, gripper_command_action_server, moveit_skill_server]")
    )
    ld.add_action(node_names_arg)
    bond_timeout_arg = DeclareLaunchArgument(
        "bond_timeout", default_value=TextSubstitution(text="100.0")
    )
    ld.add_action(bond_timeout_arg)

    marker_publisher = Node(
        package="aruco_ros",
        executable="marker_publisher",
        prefix = 'xterm -e',
        output='screen',
        name="marker_publisher",
        remappings=[
          ("camera_info", "/camera/realsense_camera_driver/color/camera_info"),
          ("image", "/camera/realsense_camera_driver/color/image_raw")]
        ,
parameters=[{
        "camera_frame": LaunchConfiguration("camera_frame"),
        "reference_frame": LaunchConfiguration("reference_frame"),
        "marker_size": LaunchConfiguration("marker_size_arg"),
        "raw_image_topic": LaunchConfiguration("raw_image_topic"),
        "camera_info_topic": LaunchConfiguration("camera_info_topic"),}]
    )
    realsense_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        prefix = 'xterm -e',
        output='screen',
        name="realsense_tf_node",
        parameters = [realsense_tf_node_config]
    )
    realsense_camera_driver = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        prefix = 'xterm -e',
        output='screen',
        name="realsense_camera_driver",
        remappings=[
          ("color/camera_info", "/camera/realsense_camera_driver/color/camera_info"),
          ("color/image_raw", "/camera/realsense_camera_driver/color/image_raw")]
        ,
parameters=[{
        "camera_name": LaunchConfiguration("camera_name"),
        "device_type": LaunchConfiguration("device_type"),
        "publish_tf": LaunchConfiguration("publish_tf"),
        "frame_id": LaunchConfiguration("frame_id"),
        "child_frame_id": LaunchConfiguration("child_frame_link"),}]
    )
    moveit_config_server = Node(
        package="moveit_skills",
        executable="moveit_config_server",
        prefix = 'xterm -e',
        output='screen',
        name="moveit_config_server",
parameters=[{
        "robot_name": LaunchConfiguration("robot_name"),
        "moveit_config_pkg": LaunchConfiguration("moveit_config"),}]
    )
    bt_operator = Node(
        package="man2_bt_operator",
        executable="bt_operator",
        prefix = 'xterm -e',
        output='screen',
        name="bt_operator",
parameters=[{
        "current_bt_xml_filename": LaunchConfiguration("current_bt_xml_filename"),
        "current_bt_xml_name": LaunchConfiguration("current_bt_xml_name"),
        "customized_plugin_lib_names.man2_bt_skill_clients": LaunchConfiguration("man2_bt_skill_clients"),
        "customized_plugin_lib_names.ur_robot_skill_clients": LaunchConfiguration("ur_robot_skill_clients"),
        "default_plugin_lib_names": LaunchConfiguration("default_plugin_lib_names"),}]
    )
    detect_acuco_marker_action_server = Node(
        package="detect_aruco_marker_skill",
        executable="detect_aruco_marker_action_server",
        prefix = 'xterm -e',
        output='screen',
        name="detect_acuco_marker_action_server",
        parameters = [detect_acuco_marker_action_server_config]
    )
    moveit_skill_server = Node(
        package="moveit_skills",
        executable="moveit_skill_server_node",
        prefix = 'xterm -e',
        output='screen',
        name="moveit_skill_server",
        parameters = [moveit_skill_server_config]
    )
    gripper_command_action_server = Node(
        package="gripper_command_skills",
        executable="gripper_command_action_server",
        prefix = 'xterm -e',
        output='screen',
        name="gripper_command_action_server"
    )
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        prefix = 'xterm -e',
        output='screen',
        name="lifecycle_manager",
parameters=[{
        "autostart": LaunchConfiguration("autostart"),
        "node_names": LaunchConfiguration("node_names"),
        "bond_timeout": LaunchConfiguration("bond_timeout"),}]
    )
    include_prbt_cell_moveit_config= IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('prbt_cell_bringup') + '/launch/bringup.launch.py'])
    )

    ld.add_action(marker_publisher)
    ld.add_action(realsense_tf_node)
    ld.add_action(realsense_camera_driver)
    ld.add_action(moveit_config_server)
    ld.add_action(bt_operator)
    ld.add_action(detect_acuco_marker_action_server)
    ld.add_action(moveit_skill_server)
    ld.add_action(gripper_command_action_server)
    ld.add_action(lifecycle_manager)
    ld.add_action(include_prbt_cell_moveit_config)

    return ld

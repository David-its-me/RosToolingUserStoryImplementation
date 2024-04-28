from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution, TextSubstitution

def generate_launch_description():
    ld = LaunchDescription()

    robot_name_arg = DeclareLaunchArgument(
        "robot_name", default_value=TextSubstitution(text="ur5e_workcell")
    )
    ld.add_action(robot_name_arg)
    moveit_config_arg = DeclareLaunchArgument(
        "moveit_config", default_value=TextSubstitution(text="ur5e_cell_moveit_config")
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
    moveit_skill_server_config = os.path.join(
        get_package_share_directory('pick_and_place_remote_rest_calls'),
        'config',
        'moveit_skill_server.yaml'
        )
    autostart_arg = DeclareLaunchArgument(
        "autostart", default_value=TextSubstitution(text="true")
    )
    ld.add_action(autostart_arg)
    node_names_arg = DeclareLaunchArgument(
        "node_names", default_value=TextSubstitution(text="[moveit_config_server, bt_operator, ur_io_control_gripper_skill_server, moveit_skill_server]")
    )
    ld.add_action(node_names_arg)
    bond_timeout_arg = DeclareLaunchArgument(
        "bond_timeout", default_value=TextSubstitution(text="100.0")
    )
    ld.add_action(bond_timeout_arg)

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
    moveit_skill_server = Node(
        package="moveit_skills",
        executable="moveit_skill_server_node",
        prefix = 'xterm -e',
        output='screen',
        name="moveit_skill_server",
        parameters = [moveit_skill_server_config]
    )
    ur_io_control_gripper_skill_server = Node(
        package="io_control_gripper_skill",
        executable="main_ur_io_control_gripper_action_server",
        prefix = 'xterm -e',
        output='screen',
        name="ur_io_control_gripper_skill_server"
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
    include_ur5e_cell_moveit_config= IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('ur5e_cell_moveit_config') + '/launch/robot.launch.py'])
    )

    ld.add_action(moveit_config_server)
    ld.add_action(bt_operator)
    ld.add_action(moveit_skill_server)
    ld.add_action(ur_io_control_gripper_skill_server)
    ld.add_action(lifecycle_manager)
    ld.add_action(include_ur5e_cell_moveit_config)

    return ld

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
    translation_x_arg = DeclareLaunchArgument(
        "translation_x", default_value=TextSubstitution(text="-0.00640796")
    )
    ld.add_action(translation_x_arg)
    translation_y_arg = DeclareLaunchArgument(
        "translation_y", default_value=TextSubstitution(text="-0.0551158")
    )
    ld.add_action(translation_y_arg)
    translation_z_arg = DeclareLaunchArgument(
        "translation_z", default_value=TextSubstitution(text="0.00796752")
    )
    ld.add_action(translation_z_arg)
    rotation_x_arg = DeclareLaunchArgument(
        "rotation_x", default_value=TextSubstitution(text="-0.529877")
    )
    ld.add_action(rotation_x_arg)
    rotation_y_arg = DeclareLaunchArgument(
        "rotation_y", default_value=TextSubstitution(text="0.53423")
    )
    ld.add_action(rotation_y_arg)
    rotation_z_arg = DeclareLaunchArgument(
        "rotation_z", default_value=TextSubstitution(text="-0.467606")
    )
    ld.add_action(rotation_z_arg)
    rotation_w_arg = DeclareLaunchArgument(
        "rotation_w", default_value=TextSubstitution(text="-0.463867")
    )
    ld.add_action(rotation_w_arg)
    child_frame_id_arg = DeclareLaunchArgument(
        "child_frame_id", default_value=TextSubstitution(text="camera_link")
    )
    ld.add_action(child_frame_id_arg)
    frame_id_arg = DeclareLaunchArgument(
        "frame_id", default_value=TextSubstitution(text="tool0")
    )
    ld.add_action(frame_id_arg)
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
        "default_plugin_lib_names", default_value=TextSubstitution(text="[util_plugin_print_value]")
    )
    ld.add_action(default_plugin_lib_names_arg)
    bond_disable_heartbeat_timeout_arg = DeclareLaunchArgument(
        "bond_disable_heartbeat_timeout", default_value=TextSubstitution(text="true")
    )
    ld.add_action(bond_disable_heartbeat_timeout_arg)
    required_pose_num_arg = DeclareLaunchArgument(
        "required_pose_num", default_value=TextSubstitution(text="2")
    )
    ld.add_action(required_pose_num_arg)
    sub_marker_topic_arg = DeclareLaunchArgument(
        "sub_marker_topic", default_value=TextSubstitution(text="/marker")
    )
    ld.add_action(sub_marker_topic_arg)
    sub_rate_arg = DeclareLaunchArgument(
        "sub_rate", default_value=TextSubstitution(text="10")
    )
    ld.add_action(sub_rate_arg)
    timeout_arg = DeclareLaunchArgument(
        "timeout", default_value=TextSubstitution(text="5000")
    )
    ld.add_action(timeout_arg)
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value=TextSubstitution(text="false")
    )
    ld.add_action(use_sim_time_arg)
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value=TextSubstitution(text="false")
    )
    ld.add_action(use_sim_time_arg)
    plan_request_params_planning_pipeline_arg = DeclareLaunchArgument(
        "plan_request_params_planning_pipeline", default_value=TextSubstitution(text="pilz_industrial_motion_planner")
    )
    ld.add_action(plan_request_params_planning_pipeline_arg)
    plan_request_params_planner_id_arg = DeclareLaunchArgument(
        "plan_request_params_planner_id", default_value=TextSubstitution(text="PTP")
    )
    ld.add_action(plan_request_params_planner_id_arg)
    plan_request_params_planning_attempts_arg = DeclareLaunchArgument(
        "plan_request_params_planning_attempts", default_value=TextSubstitution(text="10")
    )
    ld.add_action(plan_request_params_planning_attempts_arg)
    step_size_arg = DeclareLaunchArgument(
        "step_size", default_value=TextSubstitution(text="0.005")
    )
    ld.add_action(step_size_arg)
    min_fraction_arg = DeclareLaunchArgument(
        "min_fraction", default_value=TextSubstitution(text="0.5")
    )
    ld.add_action(min_fraction_arg)
    max_acceleration_scaling_factor_arg = DeclareLaunchArgument(
        "max_acceleration_scaling_factor", default_value=TextSubstitution(text="0.1")
    )
    ld.add_action(max_acceleration_scaling_factor_arg)
    planning_time_arg = DeclareLaunchArgument(
        "planning_time", default_value=TextSubstitution(text="1.0")
    )
    ld.add_action(planning_time_arg)
    planning_pipelines_pipeline_names_arg = DeclareLaunchArgument(
        "planning_pipelines_pipeline_names", default_value=TextSubstitution(text="[ompl, pilz_industrial_motion_planner]")
    )
    ld.add_action(planning_pipelines_pipeline_names_arg)
    planning_scene_monitor_options_name_arg = DeclareLaunchArgument(
        "planning_scene_monitor_options_name", default_value=TextSubstitution(text="planning_scene_monitor")
    )
    ld.add_action(planning_scene_monitor_options_name_arg)
    planning_scene_monitor_options_robot_description_arg = DeclareLaunchArgument(
        "planning_scene_monitor_options_robot_description", default_value=TextSubstitution(text="robot_description")
    )
    ld.add_action(planning_scene_monitor_options_robot_description_arg)
    planning_scene_monitor_options_joint_state_topic_arg = DeclareLaunchArgument(
        "planning_scene_monitor_options_joint_state_topic", default_value=TextSubstitution(text="/joint_states")
    )
    ld.add_action(planning_scene_monitor_options_joint_state_topic_arg)
    planning_scene_monitor_options_attached_collision_object_topic_arg = DeclareLaunchArgument(
        "planning_scene_monitor_options_attached_collision_object_topic", default_value=TextSubstitution(text="/moveit_cpp/planning_scene_monitor")
    )
    ld.add_action(planning_scene_monitor_options_attached_collision_object_topic_arg)
    planning_scene_monitor_options_publish_planning_scene_topic_arg = DeclareLaunchArgument(
        "planning_scene_monitor_options_publish_planning_scene_topic", default_value=TextSubstitution(text="/moveit_cpp/publish_planning_scene")
    )
    ld.add_action(planning_scene_monitor_options_publish_planning_scene_topic_arg)
    planning_scene_monitor_options_monitored_planning_scene_topic_arg = DeclareLaunchArgument(
        "planning_scene_monitor_options_monitored_planning_scene_topic", default_value=TextSubstitution(text="/moveit_cpp/monitored_planning_scene")
    )
    ld.add_action(planning_scene_monitor_options_monitored_planning_scene_topic_arg)
    planning_scene_monitor_options_wait_for_initial_state_timeout_arg = DeclareLaunchArgument(
        "planning_scene_monitor_options_wait_for_initial_state_timeout", default_value=TextSubstitution(text="10.0")
    )
    ld.add_action(planning_scene_monitor_options_wait_for_initial_state_timeout_arg)
    moveit_simple_controller_manager_controller_names_arg = DeclareLaunchArgument(
        "moveit_simple_controller_manager_controller_names", default_value=TextSubstitution(text="[joint_trajectory_controller]")
    )
    ld.add_action(moveit_simple_controller_manager_controller_names_arg)
    moveit_simple_controller_joint_trajectory_controller_type_arg = DeclareLaunchArgument(
        "moveit_simple_controller_joint_trajectory_controller_type", default_value=TextSubstitution(text="FollowJointTrajectory")
    )
    ld.add_action(moveit_simple_controller_joint_trajectory_controller_type_arg)
    moveit_simple_controller_joint_trajectory_controller_action_ns_arg = DeclareLaunchArgument(
        "moveit_simple_controller_joint_trajectory_controller_action_ns", default_value=TextSubstitution(text="follow_joint_trajectory")
    )
    ld.add_action(moveit_simple_controller_joint_trajectory_controller_action_ns_arg)
    moveit_simple_controller_joint_trajectory_controller_default_arg = DeclareLaunchArgument(
        "moveit_simple_controller_joint_trajectory_controller_default", default_value=TextSubstitution(text="true")
    )
    ld.add_action(moveit_simple_controller_joint_trajectory_controller_default_arg)
    moveit_simple_controller_joint_trajectory_controller_joints_arg = DeclareLaunchArgument(
        "moveit_simple_controller_joint_trajectory_controller_joints", default_value=TextSubstitution(text="[prbt_joint_1, prbt_joint_2, prbt_joint_3, prbt_joint_4, prbt_joint_5, prbt_joint_6]")
    )
    ld.add_action(moveit_simple_controller_joint_trajectory_controller_joints_arg)
    wait_server_timeout_seconds_arg = DeclareLaunchArgument(
        "wait_server_timeout_seconds", default_value=TextSubstitution(text="10")
    )
    ld.add_action(wait_server_timeout_seconds_arg)
    autostart_arg = DeclareLaunchArgument(
        "autostart", default_value=TextSubstitution(text="true")
    )
    ld.add_action(autostart_arg)
    node_names_arg = DeclareLaunchArgument(
        "node_names", default_value=TextSubstitution(text="[moveit_config_server, detect_acuco_marker_action_server, bt_operator, ur_io_control_gripper_skill_server, moveit_skill_server]")
    )
    ld.add_action(node_names_arg)
    bond_timeout_arg = DeclareLaunchArgument(
        "bond_timeout", default_value=TextSubstitution(text="60.0")
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
        parameters=[{
        "translation.x": LaunchConfiguration("translation_x"),
        "translation.y": LaunchConfiguration("translation_y"),
        "translation.z": LaunchConfiguration("translation_z"),
        "rotation.x": LaunchConfiguration("rotation_x"),
        "rotation.y": LaunchConfiguration("rotation_y"),
        "rotation.z": LaunchConfiguration("rotation_z"),
        "rotation.w": LaunchConfiguration("rotation_w"),
        "child_frame_id": LaunchConfiguration("child_frame_id"),
        "frame_id": LaunchConfiguration("frame_id"),}]
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
        parameters=[{
        "bond_disable_heartbeat_timeout": LaunchConfiguration("bond_disable_heartbeat_timeout"),
        "required_pose_num": LaunchConfiguration("required_pose_num"),
        "sub_marker_topic": LaunchConfiguration("sub_marker_topic"),
        "sub_rate": LaunchConfiguration("sub_rate"),
        "timeout": LaunchConfiguration("timeout"),
        "use_sim_time": LaunchConfiguration("use_sim_time"),}]
    )
    moveit_skill_server = Node(
        package="moveit_skills",
        executable="moveit_skill_server_node",
        prefix = 'xterm -e',
        output='screen',
        name="moveit_skill_server",
        parameters=[{
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "plan_request_params.planning_pipeline": LaunchConfiguration("plan_request_params_planning_pipeline"),
        "plan_request_params.planner_id": LaunchConfiguration("plan_request_params_planner_id"),
        "plan_request_params.planning_attempts": LaunchConfiguration("plan_request_params_planning_attempts"),
        "step_size": LaunchConfiguration("step_size"),
        "min_fraction": LaunchConfiguration("min_fraction"),
        "plan_request_params.max_acceleration_scaling_factor": LaunchConfiguration("max_acceleration_scaling_factor"),
        "planning_time": LaunchConfiguration("planning_time"),
        "planning_pipelines.pipeline_names": LaunchConfiguration("planning_pipelines_pipeline_names"),
        "planning_scene_monitor_options.name": LaunchConfiguration("planning_scene_monitor_options_name"),
        "planning_scene_monitor_options.robot_description": LaunchConfiguration("planning_scene_monitor_options_robot_description"),
        "planning_scene_monitor_options.joint_state_topic": LaunchConfiguration("planning_scene_monitor_options_joint_state_topic"),
        "planning_scene_monitor_options.attached_collision_object_topic": LaunchConfiguration("planning_scene_monitor_options_attached_collision_object_topic"),
        "planning_scene_monitor_options.publish_planning_scene_topic": LaunchConfiguration("planning_scene_monitor_options_publish_planning_scene_topic"),
        "planning_scene_monitor_options.monitored_planning_scene_topic": LaunchConfiguration("planning_scene_monitor_options_monitored_planning_scene_topic"),
        "planning_scene_monitor_options.wait_for_initial_state_timeout": LaunchConfiguration("planning_scene_monitor_options_wait_for_initial_state_timeout"),
        "moveit_simple_controller_manager.controller_names": LaunchConfiguration("moveit_simple_controller_manager_controller_names"),
        "moveit_simple_controller_manager.joint_trajectory_controller.type": LaunchConfiguration("moveit_simple_controller_joint_trajectory_controller_type"),
        "moveit_simple_controller_manager.joint_trajectory_controller.action_ns": LaunchConfiguration("moveit_simple_controller_joint_trajectory_controller_action_ns"),
        "moveit_simple_controller_manager.joint_trajectory_controller.default": LaunchConfiguration("moveit_simple_controller_joint_trajectory_controller_default"),
        "moveit_simple_controller_manager.joint_trajectory_controller.joints": LaunchConfiguration("moveit_simple_controller_joint_trajectory_controller_joints"),
        "wait_server_timeout": LaunchConfiguration("wait_server_timeout_seconds"),}]
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
    ld.add_action(ur_io_control_gripper_skill_server)
    ld.add_action(lifecycle_manager)
    ld.add_action(include_prbt_cell_moveit_config)
    ld.add_action(camera_frame_arg)
    ld.add_action(reference_frame_arg)
    ld.add_action(marker_size_arg_arg)
    ld.add_action(raw_image_topic_arg)
    ld.add_action(camera_info_topic_arg)
    ld.add_action(translation_x_arg)
    ld.add_action(translation_y_arg)
    ld.add_action(translation_z_arg)
    ld.add_action(rotation_x_arg)
    ld.add_action(rotation_y_arg)
    ld.add_action(rotation_z_arg)
    ld.add_action(rotation_w_arg)
    ld.add_action(child_frame_id_arg)
    ld.add_action(frame_id_arg)
    ld.add_action(camera_name_arg)
    ld.add_action(device_type_arg)
    ld.add_action(publish_tf_arg)
    ld.add_action(frame_id_arg)
    ld.add_action(child_frame_link_arg)
    ld.add_action(robot_name_arg)
    ld.add_action(moveit_config_arg)
    ld.add_action(current_bt_xml_filename_arg)
    ld.add_action(current_bt_xml_name_arg)
    ld.add_action(man2_bt_skill_clients_arg)
    ld.add_action(ur_robot_skill_clients_arg)
    ld.add_action(default_plugin_lib_names_arg)
    ld.add_action(bond_disable_heartbeat_timeout_arg)
    ld.add_action(required_pose_num_arg)
    ld.add_action(sub_marker_topic_arg)
    ld.add_action(sub_rate_arg)
    ld.add_action(timeout_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(plan_request_params_planning_pipeline_arg)
    ld.add_action(plan_request_params_planner_id_arg)
    ld.add_action(plan_request_params_planning_attempts_arg)
    ld.add_action(step_size_arg)
    ld.add_action(min_fraction_arg)
    ld.add_action(max_acceleration_scaling_factor_arg)
    ld.add_action(planning_time_arg)
    ld.add_action(planning_pipelines_pipeline_names_arg)
    ld.add_action(planning_scene_monitor_options_name_arg)
    ld.add_action(planning_scene_monitor_options_robot_description_arg)
    ld.add_action(planning_scene_monitor_options_joint_state_topic_arg)
    ld.add_action(planning_scene_monitor_options_attached_collision_object_topic_arg)
    ld.add_action(planning_scene_monitor_options_publish_planning_scene_topic_arg)
    ld.add_action(planning_scene_monitor_options_monitored_planning_scene_topic_arg)
    ld.add_action(planning_scene_monitor_options_wait_for_initial_state_timeout_arg)
    ld.add_action(moveit_simple_controller_manager_controller_names_arg)
    ld.add_action(moveit_simple_controller_joint_trajectory_controller_type_arg)
    ld.add_action(moveit_simple_controller_joint_trajectory_controller_action_ns_arg)
    ld.add_action(moveit_simple_controller_joint_trajectory_controller_default_arg)
    ld.add_action(moveit_simple_controller_joint_trajectory_controller_joints_arg)
    ld.add_action(wait_server_timeout_seconds_arg)
    ld.add_action(autostart_arg)
    ld.add_action(node_names_arg)
    ld.add_action(bond_timeout_arg)

    return ld

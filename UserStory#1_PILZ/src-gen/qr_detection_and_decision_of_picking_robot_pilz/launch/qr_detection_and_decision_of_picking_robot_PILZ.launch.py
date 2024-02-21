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
        "marker_size_arg", default_value=TextSubstitution(text="0.01")
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
    current_bt_xml_filename_arg = DeclareLaunchArgument(
        "current_bt_xml_filename", default_value=TextSubstitution(text="Masterarbeit_Cloud/ma-lieb-robotics-integration/04_Umsetzung/Implementation/Behaviour Tree/SimpleMovement.xml -p run_loop:=false")
    )
    ld.add_action(current_bt_xml_filename_arg)
    current_bt_xml_name_arg = DeclareLaunchArgument(
        "current_bt_xml_name", default_value=TextSubstitution(text="MoveToQR")
    )
    ld.add_action(current_bt_xml_name_arg)
    man2_bt_skill_clients_arg = DeclareLaunchArgument(
        "man2_bt_skill_clients", default_value=TextSubstitution(text="[compute_path_to_state_client, compute_path_to_pose_client, compute_path_to_point_client, execute_trajectory_client, set_path_constraints, get_current_ik_frame_pose_client, update_pose, split_pose, create_pose, update_orientation_in_pose, detect_aruco_marker_client, print_value]")
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
    planning_pipeline_arg = DeclareLaunchArgument(
        "planning_pipeline", default_value=TextSubstitution(text="ompl")
    )
    ld.add_action(planning_pipeline_arg)
    planner_id_arg = DeclareLaunchArgument(
        "planner_id", default_value=TextSubstitution(text="RRTstarkConfigDefault")
    )
    ld.add_action(planner_id_arg)
    planning_attempts_arg = DeclareLaunchArgument(
        "planning_attempts", default_value=TextSubstitution(text="10")
    )
    ld.add_action(planning_attempts_arg)
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
    robot_description_arg = DeclareLaunchArgument(
        "robot_description", default_value=TextSubstitution(text="""<?xml version="1.0" ?><!-- =================================================================================== --><!-- |    This document was autogenerated by xacro from /home/admin326/workspaces/ros2_ws/install/ur5e_cell_moveit_config/share/ur5e_cell_moveit_config/config/ur5e_workcell.urdf.xacro | --><!-- |    EDITING THIS FILE BY HAND IS NOT RECOMMENDED                                 | --><!-- =================================================================================== --><robot name="ur5e_workcell"><!-- Specify some standard inertial calculations https://en.wikipedia.org/wiki/List_of_moments_of_inertia --><!-- These make use of xacro's mathematical functionality --><!--
            Base UR robot series xacro macro.
        
            NOTE this is NOT a URDF. It cannot directly be loaded by consumers
            expecting a flattened '.urdf' file. See the top-level '.xacro' for that
            (but note that .xacro must still be processed by the xacro command).
        
            This file models the base kinematic chain of a UR robot, which then gets
            parameterised by various configuration files to convert it into a UR3(e),
            UR5(e), UR10(e) or UR16e.
        
            NOTE the default kinematic parameters (i.e., link lengths, frame locations,
            offsets, etc) do not correspond to any particular robot. They are defaults
            only. There WILL be non-zero offsets between the Forward Kinematics results
            in TF (i.e., robot_state_publisher) and the values reported by the Teach
            Pendant.
        
            For accurate (and robot-specific) transforms, the 'kinematics_parameters_file'
            parameter MUST point to a .yaml file containing the appropriate values for
            the targeted robot.
        
            If using the UniversalRobots/Universal_Robots_ROS_Driver, follow the steps
            described in the readme of that repository to extract the kinematic
            calibration from the controller and generate the required .yaml file.
        
            Main author of the migration to yaml configs Ludovic Delval.
        
            Contributors to previous versions (in no particular order)
        
             - Denis Stogl
             - Lovro Ivanov
             - Felix Messmer
             - Kelsey Hawkins
             - Wim Meeussen
             - Shaun Edwards
             - Nadia Hammoudeh Garcia
             - Dave Hershberger
             - G. vd. Hoorn
             - Philip Long
             - Dave Coleman
             - Miguel Prada
             - Mathias Luedtke
             - Marcel Schnirring
             - Felix von Drigalski
             - Felix Exner
             - Jimmy Da Silva
             - Ajit Krisshna N L
             - Muhammad Asif Rana
          --><!--
            NOTE the macro defined in this file is NOT part of the public API of this
                  package. Users CANNOT rely on this file being available, or stored in
                  this location. Nor can they rely on the existence of the macro.
          --><material name="aluminum"><color rgba="0.5 0.5 0.5 1"/></material><material name="plastic"><color rgba="0.1 0.1 0.1 1"/></material><!-- Add world link --><link name="world"/><material name="gray"><color rgba="0.6 0.8 0.8 1"/></material><link name="visuals_link"><visual><geometry><mesh filename="package://ur5e_cell_description/meshes/Assembly_new.STL" scale="0.001 0.001 0.001"/></geometry><material name="gray"/></visual></link><link name="shield_front_link"><collision><origin rpy="0 0 0" xyz="0 0 0"/><geometry><box size="1.44 0.03 1.16"/></geometry><material name="gray"/></collision><inertial><origin rpy="0 0 0" xyz="0 0 0"/><mass value="5"/><inertia ixx="0.5610416666666665" ixy="0.0" ixz="0.0" iyy="1.4246666666666665" iyz="0.0" izz="0.8643749999999999"/></inertial></link><link name="shield_rear_link"><collision><origin rpy="0 0 0 " xyz="0 0 0"/><geometry><box size="1.44 0.03 1.16"/></geometry><material name="gray"/></collision><inertial><origin rpy="0 0 0" xyz="0 0 0"/><mass value="5"/><inertia ixx="0.5610416666666665" ixy="0.0" ixz="0.0" iyy="1.4246666666666665" iyz="0.0" izz="0.8643749999999999"/></inertial></link><link name="shield_right_link"><collision><origin rpy="0 0 0" xyz="0 0 0"/><geometry><box size="0.03 0.94 1.16"/></geometry><material name="gray"/></collision><inertial><origin rpy="0 0 0" xyz="0 0 0"/><mass value="5"/><inertia ixx="0.9288333333333331" ixy="0.0" ixz="0.0" iyy="0.5610416666666665" iyz="0.0" izz="0.3685416666666666"/></inertial></link><link name="shield_left_link"><collision><origin rpy="0 0 0" xyz="0 0 0"/><geometry><box size="0.03 0.94 1.16"/></geometry><material name="gray"/></collision><inertial><origin rpy="0 0 0" xyz="0 0 0"/><mass value="5"/><inertia ixx="0.9288333333333331" ixy="0.0" ixz="0.0" iyy="0.5610416666666665" iyz="0.0" izz="0.3685416666666666"/></inertial></link><link name="robotcell_base_link"><collision><origin rpy="0 0 0" xyz="0 0 0.495"/><geometry><box size="1.44 0.94 0.99"/></geometry><material name="gray"/></collision><inertial><origin rpy="0 0 0" xyz="0 0 0"/><mass value="70"/><inertia ixx="10.871583333333332" ixy="0.0" ixz="0.0" iyy="17.81325" iyz="0.0" izz="17.25033333333333"/></inertial></link><link name="robot_link"/><joint name="robotcell_base_to_visual" type="fixed"><parent link="robotcell_base_link"/><child link="visuals_link"/><origin rpy="0 0 1.5707963267948966" xyz="0.75 -0.5 0"/></joint><joint name="robotcell_base_to_front_joint" type="fixed"><parent link="robotcell_base_link"/><child link="shield_front_link"/><origin rpy="0 0 0" xyz="0 0.45499999999999996 1.5699999999999998"/></joint><joint name="robotcell_base_to_rear" type="fixed"><parent link="robotcell_base_link"/><child link="shield_rear_link"/><origin rpy="0 0 0" xyz="0 -0.45499999999999996 1.5699999999999998"/></joint><joint name="robotcell_base_to_left" type="fixed"><parent link="robotcell_base_link"/><child link="shield_left_link"/><origin rpy="0 0 0" xyz="-0.705 0 1.5699999999999998"/></joint><joint name="robotcell_base_to_right" type="fixed"><parent link="robotcell_base_link"/><child link="shield_right_link"/><origin rpy="0 0 0" xyz="0.705 0 1.5699999999999998"/></joint><joint name="robot_cell_base_joint" type="fixed"><parent link="world"/><child link="robotcell_base_link"/><origin rpy="0 0 0" xyz="0 0 0"/></joint><joint name="robot_cell_base_to_robot_link" type="fixed"><parent link="robotcell_base_link"/><child link="robot_link"/><origin rpy="0 0 0" xyz="0.42 0.02 0.99"/></joint><ros2_control name="ur5e_workcell" type="system"><hardware><plugin>mock_components/GenericSystem</plugin><param name="fake_sensor_commands">True</param><param name="state_following_offset">0.0</param></hardware><joint name="shoulder_pan_joint"><command_interface name="position"><param name="min">{-2*pi}</param><param name="max">{2*pi}</param></command_interface><command_interface name="velocity"><param name="min">-3.15</param><param name="max">3.15</param></command_interface><state_interface name="position"><!-- initial position for the FakeSystem and simulation --><param name="initial_value">-1.57</param></state_interface><state_interface name="velocity"/><state_interface name="effort"/></joint><joint name="shoulder_lift_joint"><command_interface name="position"><param name="min">{-2*pi}</param><param name="max">{2*pi}</param></command_interface><command_interface name="velocity"><param name="min">-3.15</param><param name="max">3.15</param></command_interface><state_interface name="position"><!-- initial position for the FakeSystem and simulation --><param name="initial_value">-1.57</param></state_interface><state_interface name="velocity"/><state_interface name="effort"/></joint><joint name="elbow_joint"><command_interface name="position"><param name="min">{-pi}</param><param name="max">{pi}</param></command_interface><command_interface name="velocity"><param name="min">-3.15</param><param name="max">3.15</param></command_interface><state_interface name="position"><!-- initial position for the FakeSystem and simulation --><param name="initial_value">-1.57</param></state_interface><state_interface name="velocity"/><state_interface name="effort"/></joint><joint name="wrist_1_joint"><command_interface name="position"><param name="min">{-2*pi}</param><param name="max">{2*pi}</param></command_interface><command_interface name="velocity"><param name="min">-3.2</param><param name="max">3.2</param></command_interface><state_interface name="position"><!-- initial position for the FakeSystem and simulation --><param name="initial_value">-1.57</param></state_interface><state_interface name="velocity"/><state_interface name="effort"/></joint><joint name="wrist_2_joint"><command_interface name="position"><param name="min">{-2*pi}</param><param name="max">{2*pi}</param></command_interface><command_interface name="velocity"><param name="min">-3.2</param><param name="max">3.2</param></command_interface><state_interface name="position"><!-- initial position for the FakeSystem and simulation --><param name="initial_value">1.57</param></state_interface><state_interface name="velocity"/><state_interface name="effort"/></joint><joint name="wrist_3_joint"><command_interface name="position"><param name="min">{-2*pi}</param><param name="max">{2*pi}</param></command_interface><command_interface name="velocity"><param name="min">-3.2</param><param name="max">3.2</param></command_interface><state_interface name="position"><!-- initial position for the FakeSystem and simulation --><param name="initial_value">0.0</param></state_interface><state_interface name="velocity"/><state_interface name="effort"/></joint><sensor name="tcp_fts_sensor"><state_interface name="force.x"/><state_interface name="force.y"/><state_interface name="force.z"/><state_interface name="torque.x"/><state_interface name="torque.y"/><state_interface name="torque.z"/></sensor><!-- NOTE The following are joints used only for testing with fake hardware and will change in the future --><gpio name="speed_scaling"><state_interface name="speed_scaling_factor"/><param name="initial_speed_scaling_factor">1</param><command_interface name="target_speed_fraction_cmd"/><command_interface name="target_speed_fraction_async_success"/></gpio><gpio name="gpio"><command_interface name="standard_digital_output_cmd_0"/><command_interface name="standard_digital_output_cmd_1"/><command_interface name="standard_digital_output_cmd_2"/><command_interface name="standard_digital_output_cmd_3"/><command_interface name="standard_digital_output_cmd_4"/><command_interface name="standard_digital_output_cmd_5"/><command_interface name="standard_digital_output_cmd_6"/><command_interface name="standard_digital_output_cmd_7"/><command_interface name="standard_digital_output_cmd_8"/><command_interface name="standard_digital_output_cmd_9"/><command_interface name="standard_digital_output_cmd_10"/><command_interface name="standard_digital_output_cmd_11"/><command_interface name="standard_digital_output_cmd_12"/><command_interface name="standard_digital_output_cmd_13"/><command_interface name="standard_digital_output_cmd_14"/><command_interface name="standard_digital_output_cmd_15"/><command_interface name="standard_digital_output_cmd_16"/><command_interface name="standard_digital_output_cmd_17"/><command_interface name="standard_analog_output_cmd_0"/><command_interface name="standard_analog_output_cmd_1"/><command_interface name="tool_voltage_cmd"/><command_interface name="io_async_success"/><state_interface name="digital_output_0"/><state_interface name="digital_output_1"/><state_interface name="digital_output_2"/><state_interface name="digital_output_3"/><state_interface name="digital_output_4"/><state_interface name="digital_output_5"/><state_interface name="digital_output_6"/><state_interface name="digital_output_7"/><state_interface name="digital_output_8"/><state_interface name="digital_output_9"/><state_interface name="digital_output_10"/><state_interface name="digital_output_11"/><state_interface name="digital_output_12"/><state_interface name="digital_output_13"/><state_interface name="digital_output_14"/><state_interface name="digital_output_15"/><state_interface name="digital_output_16"/><state_interface name="digital_output_17"/><state_interface name="digital_input_0"/><state_interface name="digital_input_1"/><state_interface name="digital_input_2"/><state_interface name="digital_input_3"/><state_interface name="digital_input_4"/><state_interface name="digital_input_5"/><state_interface name="digital_input_6"/><state_interface name="digital_input_7"/><state_interface name="digital_input_8"/><state_interface name="digital_input_9"/><state_interface name="digital_input_10"/><state_interface name="digital_input_11"/><state_interface name="digital_input_12"/><state_interface name="digital_input_13"/><state_interface name="digital_input_14"/><state_interface name="digital_input_15"/><state_interface name="digital_input_16"/><state_interface name="digital_input_17"/><state_interface name="standard_analog_output_0"/><state_interface name="standard_analog_output_1"/><state_interface name="standard_analog_input_0"/><state_interface name="standard_analog_input_1"/><state_interface name="analog_io_type_0"/><state_interface name="analog_io_type_1"/><state_interface name="analog_io_type_2"/><state_interface name="analog_io_type_3"/><state_interface name="tool_mode"/><state_interface name="tool_output_voltage"/><state_interface name="tool_output_current"/><state_interface name="tool_temperature"/><state_interface name="tool_analog_input_0"/><state_interface name="tool_analog_input_1"/><state_interface name="tool_analog_input_type_0"/><state_interface name="tool_analog_input_type_1"/><state_interface name="robot_mode"/><state_interface name="robot_status_bit_0"/><state_interface name="robot_status_bit_1"/><state_interface name="robot_status_bit_2"/><state_interface name="robot_status_bit_3"/><state_interface name="safety_mode"/><state_interface name="safety_status_bit_0"/><state_interface name="safety_status_bit_1"/><state_interface name="safety_status_bit_2"/><state_interface name="safety_status_bit_3"/><state_interface name="safety_status_bit_4"/><state_interface name="safety_status_bit_5"/><state_interface name="safety_status_bit_6"/><state_interface name="safety_status_bit_7"/><state_interface name="safety_status_bit_8"/><state_interface name="safety_status_bit_9"/><state_interface name="safety_status_bit_10"/><state_interface name="program_running"/></gpio><gpio name="payload"><command_interface name="mass"/><command_interface name="cog.x"/><command_interface name="cog.y"/><command_interface name="cog.z"/><command_interface name="payload_async_success"/></gpio><gpio name="resend_robot_program"><command_interface name="resend_robot_program_cmd"/><command_interface name="resend_robot_program_async_success"/></gpio><gpio name="hand_back_control"><command_interface name="hand_back_control_cmd"/><command_interface name="hand_back_control_async_success"/></gpio><gpio name="zero_ftsensor"><command_interface name="zero_ftsensor_cmd"/><command_interface name="zero_ftsensor_async_success"/></gpio><gpio name="system_interface"><state_interface name="initialized"/></gpio></ros2_control><!-- Add URDF transmission elements (for ros_control) --><!--<xacro:ur_arm_transmission prefix="${prefix}" hw_interface="${transmission_hw_interface}" />--><!-- Placeholder for ros2_control transmission which don't yet exist --><!-- links -  main serial chain --><link name="base_link"/><link name="base_link_inertia"><visual><origin rpy="0 0 3.141592653589793" xyz="0 0 0"/><geometry><mesh filename="package://ur_description/meshes/ur5e/visual/base.dae"/></geometry></visual><collision><origin rpy="0 0 3.141592653589793" xyz="0 0 0"/><geometry><mesh filename="package://ur_description/meshes/ur5e/collision/base.stl"/></geometry></collision><inertial><mass value="4.0"/><origin rpy="0 0 0" xyz="0 0 0"/><inertia ixx="0.00443333156" ixy="0.0" ixz="0.0" iyy="0.00443333156" iyz="0.0" izz="0.0072"/></inertial></link><link name="shoulder_link"><visual><origin rpy="0 0 3.141592653589793" xyz="0 0 0"/><geometry><mesh filename="package://ur_description/meshes/ur5e/visual/shoulder.dae"/></geometry></visual><collision><origin rpy="0 0 3.141592653589793" xyz="0 0 0"/><geometry><mesh filename="package://ur_description/meshes/ur5e/collision/shoulder.stl"/></geometry></collision><inertial><mass value="3.7"/><origin rpy="0 0 0" xyz="0 0 0"/><inertia ixx="0.010267495893" ixy="0.0" ixz="0.0" iyy="0.010267495893" iyz="0.0" izz="0.00666"/></inertial></link><link name="upper_arm_link"><visual><origin rpy="1.5707963267948966 0 -1.5707963267948966" xyz="0 0 0.138"/><geometry><mesh filename="package://ur_description/meshes/ur5e/visual/upperarm.dae"/></geometry></visual><collision><origin rpy="1.5707963267948966 0 -1.5707963267948966" xyz="0 0 0.138"/><geometry><mesh filename="package://ur_description/meshes/ur5e/collision/upperarm.stl"/></geometry></collision><inertial><mass value="8.393"/><origin rpy="0 1.5707963267948966 0" xyz="-0.2125 0.0 0.138"/><inertia ixx="0.1338857818623325" ixy="0.0" ixz="0.0" iyy="0.1338857818623325" iyz="0.0" izz="0.0151074"/></inertial></link><link name="forearm_link"><visual><origin rpy="1.5707963267948966 0 -1.5707963267948966" xyz="0 0 0.007"/><geometry><mesh filename="package://ur_description/meshes/ur5e/visual/forearm.dae"/></geometry></visual><collision><origin rpy="1.5707963267948966 0 -1.5707963267948966" xyz="0 0 0.007"/><geometry><mesh filename="package://ur_description/meshes/ur5e/collision/forearm.stl"/></geometry></collision><inertial><mass value="2.275"/><origin rpy="0 1.5707963267948966 0" xyz="-0.1961 0.0 0.007"/><inertia ixx="0.031209355099586295" ixy="0.0" ixz="0.0" iyy="0.031209355099586295" iyz="0.0" izz="0.004095"/></inertial></link><link name="wrist_1_link"><visual><origin rpy="1.5707963267948966 0 0" xyz="0 0 -0.127"/><geometry><mesh filename="package://ur_description/meshes/ur5e/visual/wrist1.dae"/></geometry></visual><collision><origin rpy="1.5707963267948966 0 0" xyz="0 0 -0.127"/><geometry><mesh filename="package://ur_description/meshes/ur5e/collision/wrist1.stl"/></geometry></collision><inertial><mass value="1.219"/><origin rpy="0 0 0" xyz="0 0 0"/><inertia ixx="0.0025598989760400002" ixy="0.0" ixz="0.0" iyy="0.0025598989760400002" iyz="0.0" izz="0.0021942"/></inertial></link><link name="wrist_2_link"><visual><origin rpy="0 0 0" xyz="0 0 -0.0997"/><geometry><mesh filename="package://ur_description/meshes/ur5e/visual/wrist2.dae"/></geometry></visual><collision><origin rpy="0 0 0" xyz="0 0 -0.0997"/><geometry><mesh filename="package://ur_description/meshes/ur5e/collision/wrist2.stl"/></geometry></collision><inertial><mass value="1.219"/><origin rpy="0 0 0" xyz="0 0 0"/><inertia ixx="0.0025598989760400002" ixy="0.0" ixz="0.0" iyy="0.0025598989760400002" iyz="0.0" izz="0.0021942"/></inertial></link><link name="wrist_3_link"><visual><origin rpy="1.5707963267948966 0 0" xyz="0 0 -0.0989"/><geometry><mesh filename="package://ur_description/meshes/ur5e/visual/wrist3.dae"/></geometry></visual><collision><origin rpy="1.5707963267948966 0 0" xyz="0 0 -0.0989"/><geometry><mesh filename="package://ur_description/meshes/ur5e/collision/wrist3.stl"/></geometry></collision><inertial><mass value="0.1879"/><origin rpy="0 0 0" xyz="0.0 0.0 -0.0229"/><inertia ixx="9.890410052167731e-05" ixy="0.0" ixz="0.0" iyy="9.890410052167731e-05" iyz="0.0" izz="0.0001321171875"/></inertial></link><!-- base_joint fixes base_link to the environment --><joint name="base_joint" type="fixed"><origin rpy="0 0 1.5707963267948966" xyz="0 0 0"/><parent link="robot_link"/><child link="base_link"/></joint><!-- joints - main serial chain --><joint name="base_link-base_link_inertia" type="fixed"><parent link="base_link"/><child link="base_link_inertia"/><!-- 'base_link' is REP-103 aligned (so X+ forward), while the internal
                   frames of the robot/controller have X+ pointing backwards.
                   Use the joint between 'base_link' and 'base_link_inertia' (a dummy
                   link/frame) to introduce the necessary rotation over Z (of pi rad).
              --><origin rpy="0 0 3.141592653589793" xyz="0 0 0"/></joint><joint name="shoulder_pan_joint" type="revolute"><parent link="base_link_inertia"/><child link="shoulder_link"/><origin rpy="0 0 4.129148531428761e-08" xyz="0 0 0.16257940898924925"/><axis xyz="0 0 1"/><limit effort="150.0" lower="-6.283185307179586" upper="6.283185307179586" velocity="3.141592653589793"/><dynamics damping="0" friction="0"/></joint><joint name="shoulder_lift_joint" type="revolute"><parent link="shoulder_link"/><child link="upper_arm_link"/><origin rpy="1.569869765796217 0 5.036318121915316e-06" xyz="4.928395535654581e-05 0 0"/><axis xyz="0 0 1"/><limit effort="150.0" lower="-6.283185307179586" upper="6.283185307179586" velocity="3.141592653589793"/><dynamics damping="0" friction="0"/></joint><joint name="elbow_joint" type="revolute"><parent link="upper_arm_link"/><child link="forearm_link"/><origin rpy="3.1392013464244997 3.1391688059969463 -3.1415916373618824" xyz="-0.4254523763434618 0 0"/><axis xyz="0 0 1"/><limit effort="150.0" lower="-3.141592653589793" upper="0.0" velocity="3.141592653589793"/><dynamics damping="0" friction="0"/></joint><joint name="wrist_1_joint" type="revolute"><parent link="forearm_link"/><child link="wrist_1_link"/><origin rpy="3.1407543838911054 -3.1393367089449136 3.141588769544439" xyz="-0.3925179308036957 0.00011209320964041982 0.13371971283234207"/><axis xyz="0 0 1"/><limit effort="28.0" lower="-6.283185307179586" upper="6.283185307179586" velocity="3.141592653589793"/><dynamics damping="0" friction="0"/></joint><joint name="wrist_2_joint" type="revolute"><parent link="wrist_1_link"/><child link="wrist_2_link"/><origin rpy="1.5696737491576425 0 1.6248900383403209e-06" xyz="-5.074257472996066e-05 -0.09963845508736849 0.0001118519484761866"/><axis xyz="0 0 1"/><limit effort="28.0" lower="-6.283185307179586" upper="6.283185307179586" velocity="3.141592653589793"/><dynamics damping="0" friction="0"/></joint><joint name="wrist_3_joint" type="revolute"><parent link="wrist_2_link"/><child link="wrist_3_link"/><origin rpy="1.5724268955944378 3.141592653589793 3.141592586202488" xyz="0.00015530393797499986 0.09948201802868764 0.00016221241847401644"/><axis xyz="0 0 1"/><limit effort="28.0" lower="-6.283185307179586" upper="6.283185307179586" velocity="3.141592653589793"/><dynamics damping="0" friction="0"/></joint><link name="ft_frame"/><joint name="wrist_3_link-ft_frame" type="fixed"><parent link="wrist_3_link"/><child link="ft_frame"/><origin rpy="3.141592653589793 0 0" xyz="0 0 0"/></joint><!-- ROS-Industrial 'base' frame - base_link to UR 'Base' Coordinates transform --><link name="base"/><joint name="base_link-base_fixed_joint" type="fixed"><!-- Note the rotation over Z of pi radians - as base_link is REP-103
                   aligned (i.e., has X+ forward, Y+ left and Z+ up), this is needed
                   to correctly align 'base' with the 'Base' coordinate system of
                   the UR controller.
              --><origin rpy="0 0 3.141592653589793" xyz="0 0 0"/><parent link="base_link"/><child link="base"/></joint><!-- ROS-Industrial 'flange' frame - attachment point for EEF models --><link name="flange"/><joint name="wrist_3-flange" type="fixed"><parent link="wrist_3_link"/><child link="flange"/><origin rpy="0 -1.5707963267948966 -1.5707963267948966" xyz="0 0 0"/></joint><!-- ROS-Industrial 'tool0' frame - all-zeros tool frame --><link name="tool0"/><joint name="flange-tool0" type="fixed"><!-- default toolframe - X+ left, Y+ up, Z+ front --><origin rpy="1.5707963267948966 0 1.5707963267948966" xyz="0 0 0"/><parent link="flange"/><child link="tool0"/></joint><!-- Specify some standard inertial calculations https://en.wikipedia.org/wiki/List_of_moments_of_inertia --><!-- These make use of xacro's mathematical functionality --><material name="blue"><color rgba="0 0.2 1 1"/></material><material name="green"><color rgba="0 0.9 0.3 1"/></material><material name="white"><color rgba="1 1 1 1"/></material><!--Base plate--><link name="schunk_egp50_base_link"><visual><geometry><cylinder length="0.015" radius="0.0375"/></geometry><material name="blue"/><origin rpy="0 0 0" xyz="0 0 0.0075"/></visual><collision><geometry><cylinder length="0.015" radius="0.0375"/></geometry><origin rpy="0 0 0" xyz="0 0 0.0075"/></collision><inertial><origin rpy="0 0 0" xyz="0 0 0.0075"/><mass value="0.2"/><inertia ixx="7.406249999999999e-05" ixy="0.0" ixz="0.0" iyy="7.406249999999999e-05" iyz="0.0" izz="0.000140625"/></inertial></link><!--Main gripper--><link name="schunk_egp50_body_link"><visual><geometry><mesh filename="package://ur5e_cell_description/meshes/Gripper.stl" scale="0.001 0.001 0.001"/></geometry><origin rpy="1.5707963267948966 0 0" xyz="-0.025 0.015 -0.0433"/><material name="green"/></visual><collision><geometry><mesh filename="package://ur5e_cell_description/meshes/Gripper.stl" scale="0.001 0.001 0.001"/></geometry><origin rpy="1.5707963267948966 0 0" xyz="-0.025 0.015 -0.0433"/></collision><inertial><origin rpy="1.5707963267948966 0 0" xyz="-0.025 0.015 -0.0433"/><mass value="0.5"/><inertia ixx="0.00034998166666666666" ixy="0.0" ixz="0.0" iyy="0.00041664833333333336" iyz="0.0" izz="0.00014166666666666668"/></inertial></link><!--Pincer block 1--><link name="schunk_egp50_pincer_1_link"><visual><geometry><mesh filename="package://ur5e_cell_description/meshes/Pincer_1.stl" scale="0.001 0.001 0.001"/></geometry><origin rpy="1.5707963267948966 0 0" xyz="-0.025 0.009 -0.0936"/><material name="white"/></visual><collision><geometry><mesh filename="package://ur5e_cell_description/meshes/Pincer_1.stl" scale="0.001 0.001 0.001"/></geometry><origin rpy="1.5707963267948966 0 0" xyz="-0.025 0.009 -0.0936"/></collision><inertial><origin rpy="1.5707963267948966 0 0" xyz="-0.025 0.009 -0.0936"/><mass value="0.1"/><inertia ixx="4.333333333333333e-06" ixy="0.0" ixz="0.0" iyy="2.246666666666667e-05" iyz="0.0" izz="2.3533333333333337e-05"/></inertial></link><!--Pincer block 2--><link name="schunk_egp50_pincer_2_link"><visual><geometry><mesh filename="package://ur5e_cell_description/meshes/Pincer_2.stl" scale="0.001 0.001 0.001"/></geometry><origin rpy="1.5707963267948966 0 0" xyz="-0.025 0.009 -0.0936"/><material name="blue"/></visual><collision><geometry><mesh filename="package://ur5e_cell_description/meshes/Pincer_2.stl" scale="0.001 0.001 0.001"/></geometry><origin rpy="1.5707963267948966 0 0" xyz="-0.025 0.009 -0.0936"/></collision><inertial><origin rpy="1.5707963267948966 0 0" xyz="-0.025 0.009 -0.0936"/><mass value="0.1"/><inertia ixx="4.333333333333333e-06" ixy="0.0" ixz="0.0" iyy="2.246666666666667e-05" iyz="0.0" izz="2.3533333333333337e-05"/></inertial></link><!-- Virtual link for visualizytion and planning--><link name="tool_tip"/><!-- Joints--><joint name="schunk_egp50_body_joint" type="fixed"><parent link="schunk_egp50_base_link"/><child link="schunk_egp50_body_link"/><origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0583"/></joint><joint name="schunk_egp50_pincer_1_joint" type="fixed"><parent link="schunk_egp50_body_link"/><child link="schunk_egp50_pincer_1_link"/><origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0503"/><!-- the motion of the pincer is along the X axis --><axis xyz="1 0 0"/><!-- the motion of the pincer is limited to the half of the gripper body length --><limit effort="100" lower="-0.003125" upper="0.003125" velocity="100"/></joint><joint name="schunk_egp50_pincer_2_joint" type="fixed"><parent link="schunk_egp50_body_link"/><child link="schunk_egp50_pincer_2_link"/><origin rpy="0.0 0.0 0.0" xyz="0.0 0.002 0.0503"/><!-- the motion of the pincer is along the X axis --><axis xyz="1 0 0"/><!-- the motion of the pincer is limited to the half of the gripper body length --><limit effort="100" lower="0.003125" upper="-0.003125" velocity="100"/></joint><joint name="schunk_egp50_tip_joint" type="fixed"><parent link="schunk_egp50_base_link"/><child link="tool_tip"/><origin rpy="0.0 0.0 0.0" xyz="0 0 0.1044"/></joint><joint name="schunk_egp50_base_joint" type="fixed"><parent link="tool0"/><child link="schunk_egp50_base_link"/><origin rpy="0 0 0" xyz="0 0 0"/></joint><!-- camera body, with origin at bottom screw mount --><joint name="camera_joint" type="fixed"><origin rpy="1.5707963267948966 -1.5707963267948966 0" xyz="0 -0.05 0"/><parent link="tool0"/><child link="camera_bottom_screw_frame"/></joint><link name="camera_bottom_screw_frame"/><joint name="camera_link_joint" type="fixed"><origin rpy="0 0 0" xyz="0.010600000000000002 0.0175 0.0125"/><parent link="camera_bottom_screw_frame"/><child link="camera_link"/></joint><link name="camera_link"><visual><!-- the mesh origin is at front plate in between the two infrared camera axes --><origin rpy="1.5707963267948966 0 1.5707963267948966" xyz="0.0043 -0.0175 0"/><geometry><mesh filename="file:///home/admin326/workspaces/ros2_ws/install/realsense2_description/share/realsense2_description/meshes/d435.dae"/></geometry></visual><collision><origin rpy="0 0 0" xyz="0 -0.0175 0"/><geometry><box size="0.02505 0.09 0.025"/></geometry></collision><inertial><!-- The following are not reliable values, and should not be used for modeling --><mass value="0.072"/><origin xyz="0 0 0"/><inertia ixx="0.003881243" ixy="0.0" ixz="0.0" iyy="0.000498940" iyz="0.0" izz="0.003879257"/></inertial></link></robot>""")
    )
    ld.add_action(robot_description_arg)
    robot_description_semantic_arg = DeclareLaunchArgument(
        "robot_description_semantic", default_value=TextSubstitution(text=""" '<?xml version="1.0" ?><!-- =================================================================================== --><!-- |    This document was autogenerated by xacro from /home/admin326/workspaces/ros2_ws/install/ur5e_cell_moveit_config/share/ur5e_cell_moveit_config/config/ur5e_workcell.srdf | --><!-- |    EDITING THIS FILE BY HAND IS NOT RECOMMENDED                                 | --><!-- =================================================================================== --><!--This does not replace URDF, and is not an extension of URDF.
            This is a format for representing semantic information about the robot structure.
            A URDF file must exist for this robot as well, where the joints and the links that are referenced are defined
        --><robot name="ur5e_workcell"><!--GROUPS: Representation of a set of joints and links. This can be useful for specifying DOF to plan for, defining arms, end effectors, etc--><!--LINKS: When a link is specified, the parent joint of that link (if it exists) is automatically included--><!--JOINTS: When a joint is specified, the child link of that joint (which will always exist) is automatically included--><!--CHAINS: When a chain is specified, all the links along the chain (including endpoints) are included in the group. Additionally, all the joints that are parents to included links are also included. This means that joints along the chain and the parent joint of the base link are included in the group--><!--SUBGROUPS: Groups can also be formed by referencing to already defined group names--><group name="arm"><joint name="shoulder_pan_joint"/><joint name="shoulder_lift_joint"/><joint name="elbow_joint"/><joint name="wrist_1_joint"/><joint name="wrist_2_joint"/><joint name="wrist_3_joint"/><chain base_link="base_link" tip_link="tool_tip"/></group><group name="hand"><joint name="schunk_egp50_tip_joint"/><joint name="schunk_egp50_pincer_2_joint"/><joint name="schunk_egp50_pincer_1_joint"/><joint name="schunk_egp50_body_joint"/><chain base_link="schunk_egp50_base_link" tip_link="tool_tip"/></group><group_state group="arm" name="home"><joint name="elbow_joint" value="-1.5794"/><joint name="shoulder_lift_joint" value="-1.74"/><joint name="shoulder_pan_joint" value="-1.5708"/><joint name="wrist_1_joint" value="0.1736"/><joint name="wrist_2_joint" value="1.5621"/><joint name="wrist_3_joint" value="0"/></group_state><group_state group="arm" name="detect"><joint name="elbow_joint" value="-1.7193"/><joint name="shoulder_lift_joint" value="-1.5446"/><joint name="shoulder_pan_joint" value="-1.4240"/><joint name="wrist_1_joint" value="-1.44775"/><joint name="wrist_2_joint" value="1.5708"/><joint name="wrist_3_joint" value="0.15865"/></group_state><!--END EFFECTOR: Purpose: Represent information about an end effector.--><end_effector group="hand" name="hand" parent_group="arm" parent_link="tool0"/><!--DISABLE COLLISIONS: By default it is assumed that any link of the robot could potentially come into collision with any other link in the robot. This tag disables collision checking between a specified pair of links. --><disable_collisions link1="base_link_inertia" link2="robotcell_base_link" reason="Adjacent"/><disable_collisions link1="base_link_inertia" link2="shield_front_link" reason="Never"/><disable_collisions link1="base_link_inertia" link2="shield_left_link" reason="Never"/><disable_collisions link1="base_link_inertia" link2="shield_rear_link" reason="Never"/><disable_collisions link1="base_link_inertia" link2="shield_right_link" reason="Never"/><disable_collisions link1="base_link_inertia" link2="shoulder_link" reason="Adjacent"/><!-- <disable_collisions link1="camera_link" link2="shield_left_link" reason="Never"/>
            <disable_collisions link1="camera_link" link2="schunk_egp50_base_link" reason="Adjacent"/>
            <disable_collisions link1="camera_link" link2="schunk_egp50_body_link" reason="Never"/>
            <disable_collisions link1="camera_link" link2="schunk_egp50_pincer_1_link" reason="Never"/>
            <disable_collisions link1="camera_link" link2="schunk_egp50_pincer_2_link" reason="Never"/>
            <disable_collisions link1="camera_link" link2="wrist_2_link" reason="Never"/>
            <disable_collisions link1="camera_link" link2="wrist_3_link" reason="Adjacent"/> --><disable_collisions link1="forearm_link" link2="shield_front_link" reason="Default"/><disable_collisions link1="forearm_link" link2="shield_left_link" reason="Never"/><disable_collisions link1="forearm_link" link2="upper_arm_link" reason="Adjacent"/><disable_collisions link1="forearm_link" link2="wrist_1_link" reason="Adjacent"/><disable_collisions link1="forearm_link" link2="wrist_2_link" reason="Never"/><disable_collisions link1="robotcell_base_link" link2="shield_front_link" reason="Adjacent"/><disable_collisions link1="robotcell_base_link" link2="shield_left_link" reason="Adjacent"/><disable_collisions link1="robotcell_base_link" link2="shield_rear_link" reason="Adjacent"/><disable_collisions link1="robotcell_base_link" link2="shield_right_link" reason="Adjacent"/><disable_collisions link1="robotcell_base_link" link2="shoulder_link" reason="Never"/><disable_collisions link1="shield_front_link" link2="shield_left_link" reason="Default"/><disable_collisions link1="shield_front_link" link2="shield_rear_link" reason="Never"/><disable_collisions link1="shield_front_link" link2="shield_right_link" reason="Default"/><disable_collisions link1="shield_front_link" link2="shoulder_link" reason="Never"/><disable_collisions link1="shield_front_link" link2="upper_arm_link" reason="Default"/><disable_collisions link1="shield_left_link" link2="shield_rear_link" reason="Default"/><disable_collisions link1="shield_left_link" link2="shield_right_link" reason="Never"/><disable_collisions link1="shield_left_link" link2="shoulder_link" reason="Never"/><disable_collisions link1="shield_left_link" link2="schunk_egp50_base_link" reason="Never"/><disable_collisions link1="shield_left_link" link2="schunk_egp50_body_link" reason="Never"/><disable_collisions link1="shield_left_link" link2="schunk_egp50_pincer_1_link" reason="Never"/><disable_collisions link1="shield_left_link" link2="schunk_egp50_pincer_2_link" reason="Never"/><disable_collisions link1="shield_left_link" link2="upper_arm_link" reason="Never"/><disable_collisions link1="shield_left_link" link2="wrist_1_link" reason="Never"/><disable_collisions link1="shield_left_link" link2="wrist_2_link" reason="Never"/><disable_collisions link1="shield_left_link" link2="wrist_3_link" reason="Never"/><disable_collisions link1="shield_rear_link" link2="shield_right_link" reason="Default"/><disable_collisions link1="shield_rear_link" link2="shoulder_link" reason="Never"/><disable_collisions link1="shield_right_link" link2="shoulder_link" reason="Never"/><disable_collisions link1="shoulder_link" link2="upper_arm_link" reason="Adjacent"/><disable_collisions link1="schunk_egp50_base_link" link2="schunk_egp50_body_link" reason="Adjacent"/><disable_collisions link1="schunk_egp50_base_link" link2="schunk_egp50_pincer_1_link" reason="Never"/><disable_collisions link1="schunk_egp50_base_link" link2="schunk_egp50_pincer_2_link" reason="Never"/><disable_collisions link1="schunk_egp50_base_link" link2="wrist_1_link" reason="Never"/><disable_collisions link1="schunk_egp50_base_link" link2="wrist_2_link" reason="Never"/><disable_collisions link1="schunk_egp50_base_link" link2="wrist_3_link" reason="Adjacent"/><disable_collisions link1="schunk_egp50_body_link" link2="schunk_egp50_pincer_1_link" reason="Adjacent"/><disable_collisions link1="schunk_egp50_body_link" link2="schunk_egp50_pincer_2_link" reason="Adjacent"/><disable_collisions link1="schunk_egp50_body_link" link2="wrist_1_link" reason="Never"/><disable_collisions link1="schunk_egp50_body_link" link2="wrist_2_link" reason="Never"/><disable_collisions link1="schunk_egp50_body_link" link2="wrist_3_link" reason="Never"/><disable_collisions link1="schunk_egp50_pincer_1_link" link2="schunk_egp50_pincer_2_link" reason="Never"/><disable_collisions link1="schunk_egp50_pincer_1_link" link2="wrist_1_link" reason="Never"/><disable_collisions link1="schunk_egp50_pincer_1_link" link2="wrist_2_link" reason="Never"/><disable_collisions link1="schunk_egp50_pincer_1_link" link2="wrist_3_link" reason="Never"/><disable_collisions link1="schunk_egp50_pincer_2_link" link2="wrist_1_link" reason="Never"/><disable_collisions link1="schunk_egp50_pincer_2_link" link2="wrist_2_link" reason="Never"/><disable_collisions link1="schunk_egp50_pincer_2_link" link2="wrist_3_link" reason="Never"/><disable_collisions link1="wrist_1_link" link2="wrist_2_link" reason="Adjacent"/><disable_collisions link1="wrist_1_link" link2="wrist_3_link" reason="Never"/><disable_collisions link1="wrist_2_link" link2="wrist_3_link" reason="Adjacent"/></robot>' """)
    )
    ld.add_action(robot_description_semantic_arg)
    kinematics_solver_arg = DeclareLaunchArgument(
        "kinematics_solver", default_value=TextSubstitution(text="kdl_kinematics_plugin/KDLKinematicsPlugin")
    )
    ld.add_action(kinematics_solver_arg)
    kinematics_solver_search_resolution_arg = DeclareLaunchArgument(
        "kinematics_solver_search_resolution", default_value=TextSubstitution(text="0.005")
    )
    ld.add_action(kinematics_solver_search_resolution_arg)
    kinematics_solver_timeout_arg = DeclareLaunchArgument(
        "kinematics_solver_timeout", default_value=TextSubstitution(text="0.005")
    )
    ld.add_action(kinematics_solver_timeout_arg)
    pipeline_names_arg = DeclareLaunchArgument(
        "pipeline_names", default_value=TextSubstitution(text="[pilz_industrial_motion_planner]")
    )
    ld.add_action(pipeline_names_arg)
    pilz_industrial_motion_planner_planning_plugin_arg = DeclareLaunchArgument(
        "pilz_industrial_motion_planner_planning_plugin", default_value=TextSubstitution(text="pilz_industrial_motion_planner/CommandPlanner")
    )
    ld.add_action(pilz_industrial_motion_planner_planning_plugin_arg)
    pilz_industrial_motion_planner_request_adapters_arg = DeclareLaunchArgument(
        "pilz_industrial_motion_planner_request_adapters", default_value=TextSubstitution(text="default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints")
    )
    ld.add_action(pilz_industrial_motion_planner_request_adapters_arg)
    pilz_industrial_motion_planner_default_planner_config_arg = DeclareLaunchArgument(
        "pilz_industrial_motion_planner_default_planner_config", default_value=TextSubstitution(text="PTP")
    )
    ld.add_action(pilz_industrial_motion_planner_default_planner_config_arg)
    pilz_industrial_motion_capabilities_arg = DeclareLaunchArgument(
        "pilz_industrial_motion_capabilities", default_value=TextSubstitution(text="pilz_industrial_motion_planner/MoveGroupSequenceAction pilz_industrial_motion_planner/MoveGroupSequenceService")
    )
    ld.add_action(pilz_industrial_motion_capabilities_arg)
    planning_scene_monitor_options_name_arg = DeclareLaunchArgument(
        "planning_scene_monitor_options_name", default_value=TextSubstitution(text="planning_scene_monitor")
    )
    ld.add_action(planning_scene_monitor_options_name_arg)
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
    planning_scene_monitor_options_name_arg = DeclareLaunchArgument(
        "planning_scene_monitor_options_name", default_value=TextSubstitution(text="planning_scene_monitor")
    )
    ld.add_action(planning_scene_monitor_options_name_arg)
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
    planning_scene_monitor_options_monitored_planning_scene_topic_arg = DeclareLaunchArgument(
        "planning_scene_monitor_options_monitored_planning_scene_topic", default_value=TextSubstitution(text="10.0")
    )
    ld.add_action(planning_scene_monitor_options_monitored_planning_scene_topic_arg)
    autostart_arg = DeclareLaunchArgument(
        "autostart", default_value=TextSubstitution(text="true")
    )
    ld.add_action(autostart_arg)
    node_names_arg = DeclareLaunchArgument(
        "node_names", default_value=TextSubstitution(text="[bt_operator, detect_aruco_marker_action_server, moveit_skill_server]")
    )
    ld.add_action(node_names_arg)
    bond_timeout_arg = DeclareLaunchArgument(
        "bond_timeout", default_value=TextSubstitution(text="4.0")
    )
    ld.add_action(bond_timeout_arg)

    marker_publisher = Node(
        package="aruco_ros",
        executable="marker_publisher",
        prefix = 'xterm -e',
        output='screen',
        name="marker_publisher",
        remappings=[
          ("camera_info", "/camera_info"),
          ("image", "/camera_image")]
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
          ("color/camera_info", "/camera_info"),
          ("color/image_raw", "/camera_image")]
        ,
        parameters=[{
        "camera_name": LaunchConfiguration("camera_name"),
        "device_type": LaunchConfiguration("device_type"),
        "publish_tf": LaunchConfiguration("publish_tf"),
        "frame_id": LaunchConfiguration("frame_id"),
        "child_frame_id": LaunchConfiguration("child_frame_link"),}]
    )
    bt_operator = Node(
        package="man2_bt_operator",
        executable="bt_operator",
        prefix = 'xterm -e',
        output='screen',
        name="bt_operator",
        remappings=[
          ("~/get_state", "get_state_bt_operator_server"),
          ("~/change_state", "change_state_bt_operator_server")]
        ,
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
        remappings=[
          ("~/get_state", "get_state_aruco_marker_action_server"),
          ("~/change_state", "change_state_aruco_marker_action_server")]
        ,
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
        "planning_pipeline": LaunchConfiguration("planning_pipeline"),
        "planner_id": LaunchConfiguration("planner_id"),
        "planning_attempts": LaunchConfiguration("planning_attempts"),
        "step_size": LaunchConfiguration("step_size"),
        "min_fraction": LaunchConfiguration("min_fraction"),
        "max_acceleration_scaling_factor": LaunchConfiguration("max_acceleration_scaling_factor"),
        "planning_time": LaunchConfiguration("planning_time"),
        "robot_description": LaunchConfiguration("robot_description"),
        "robot_description_semantic": LaunchConfiguration("robot_description_semantic"),
        "robot_description_kinematics/arm/kinematics_solver": LaunchConfiguration("kinematics_solver"),
        "robot_description_kinematics/arm/kinematics_solver_search_resolution": LaunchConfiguration("kinematics_solver_search_resolution"),
        "robot_description_kinematics/arm/kinematics_solver_timeout": LaunchConfiguration("kinematics_solver_timeout"),
        "planning_pipelines/pipeline_names": LaunchConfiguration("pipeline_names"),
        "pilz_industrial_motion_planner/planning_plugin": LaunchConfiguration("pilz_industrial_motion_planner_planning_plugin"),
        "pilz_industrial_motion_planner/request_adapters": LaunchConfiguration("pilz_industrial_motion_planner_request_adapters"),
        "pilz_industrial_motion_planner/default_planner_config": LaunchConfiguration("pilz_industrial_motion_planner_default_planner_config"),
        "pilz_industrial_motion_planner/capabilities": LaunchConfiguration("pilz_industrial_motion_capabilities"),
        "planning_scene_monitor_options/name": LaunchConfiguration("planning_scene_monitor_options_name"),
        "planning_scene_monitor_options/name": LaunchConfiguration("planning_scene_monitor_options_name"),
        "planning_scene_monitor_options/robot_description": LaunchConfiguration("planning_scene_monitor_options_robot_description"),
        "planning_scene_monitor_options/joint_state_topic": LaunchConfiguration("planning_scene_monitor_options_joint_state_topic"),
        "planning_scene_monitor_options/name": LaunchConfiguration("planning_scene_monitor_options_name"),
        "planning_scene_monitor_options/attached_collision_object_topic": LaunchConfiguration("planning_scene_monitor_options_attached_collision_object_topic"),
        "planning_scene_monitor_options/publish_planning_scene_topic": LaunchConfiguration("planning_scene_monitor_options_publish_planning_scene_topic"),
        "planning_scene_monitor_options/monitored_planning_scene_topic": LaunchConfiguration("planning_scene_monitor_options_monitored_planning_scene_topic"),
        "planning_scene_monitor_options/wait_for_initial_state_timeout": LaunchConfiguration("planning_scene_monitor_options_monitored_planning_scene_topic"),}]
    )
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        prefix = 'xterm -e',
        output='screen',
        name="lifecycle_manager",
        remappings=[
          ("detect_aruco_marker_action_server/get_state", "get_state_aruco_marker_action_server"),
          ("detect_aruco_marker_action_server/change_state", "change_state_aruco_marker_action_server"),
          ("bt_operator/get_state", "get_state_bt_operator_server"),
          ("bt_operator/change_state", "change_state_bt_operator_server")]
        ,
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
    ld.add_action(bt_operator)
    ld.add_action(detect_acuco_marker_action_server)
    ld.add_action(moveit_skill_server)
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
    ld.add_action(planning_pipeline_arg)
    ld.add_action(planner_id_arg)
    ld.add_action(planning_attempts_arg)
    ld.add_action(step_size_arg)
    ld.add_action(min_fraction_arg)
    ld.add_action(max_acceleration_scaling_factor_arg)
    ld.add_action(planning_time_arg)
    ld.add_action(robot_description_arg)
    ld.add_action(robot_description_semantic_arg)
    ld.add_action(kinematics_solver_arg)
    ld.add_action(kinematics_solver_search_resolution_arg)
    ld.add_action(kinematics_solver_timeout_arg)
    ld.add_action(pipeline_names_arg)
    ld.add_action(pilz_industrial_motion_planner_planning_plugin_arg)
    ld.add_action(pilz_industrial_motion_planner_request_adapters_arg)
    ld.add_action(pilz_industrial_motion_planner_default_planner_config_arg)
    ld.add_action(pilz_industrial_motion_capabilities_arg)
    ld.add_action(planning_scene_monitor_options_name_arg)
    ld.add_action(planning_scene_monitor_options_name_arg)
    ld.add_action(planning_scene_monitor_options_robot_description_arg)
    ld.add_action(planning_scene_monitor_options_joint_state_topic_arg)
    ld.add_action(planning_scene_monitor_options_name_arg)
    ld.add_action(planning_scene_monitor_options_attached_collision_object_topic_arg)
    ld.add_action(planning_scene_monitor_options_publish_planning_scene_topic_arg)
    ld.add_action(planning_scene_monitor_options_monitored_planning_scene_topic_arg)
    ld.add_action(planning_scene_monitor_options_monitored_planning_scene_topic_arg)
    ld.add_action(autostart_arg)
    ld.add_action(node_names_arg)
    ld.add_action(bond_timeout_arg)

    return ld

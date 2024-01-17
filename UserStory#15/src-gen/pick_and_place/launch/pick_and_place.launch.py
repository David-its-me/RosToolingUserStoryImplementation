from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        prefix = 'xterm -e',
        output='screen',
        name="robot_state_publisher_node"
    )

    controller_manager = Node(
        package="controller_manager",
        executable="controller_manager",
        prefix = 'xterm -e',
        output='screen',
        name="controller_manager"
    )

    initial_joint_controller = Node(
        package="controller_manager",
        executable="controller_manager",
        prefix = 'xterm -e',
        output='screen',
        name="initial_joint_controller"
    )

    initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="controller_manager",
        prefix = 'xterm -e',
        output='screen',
        name="initial_joint_controller_spawner_stopped"
    )

    ur_ros2_control_node = Node(
        package="ur_robot_driver",
        executable="ur_ros2_control_node",
        prefix = 'xterm -e',
        output='screen',
        name="ur_ros2_control_node"
    )

    dashboard_client = Node(
        package="ur_robot_driver",
        executable="dashboard_client",
        prefix = 'xterm -e',
        output='screen',
        name="dashboard_client"
    )

    urscript_interface = Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        prefix = 'xterm -e',
        output='screen',
        name="urscript_interface"
    )

    controller_stopper = Node(
        package="ur_robot_driver",
        executable="controller_stopper_node",
        prefix = 'xterm -e',
        output='screen',
        name="controller_stopper"
    )

    moveit_simple_controller_manager = Node(
        package="moveit_simple_controller_manager",
        executable="moveit_simple_controller_manager",
        prefix = 'xterm -e',
        output='screen',
        name="moveit_simple_controller_manager"
    )

    

    ld.add_action(robot_state_publisher_node)
    ld.add_action(controller_manager)
    ld.add_action(initial_joint_controller)
    ld.add_action(initial_joint_controller_spawner_stopped)
    ld.add_action(ur_ros2_control_node)
    ld.add_action(dashboard_client)
    ld.add_action(urscript_interface)
    ld.add_action(controller_stopper)
    ld.add_action(moveit_simple_controller_manager)

    return ld

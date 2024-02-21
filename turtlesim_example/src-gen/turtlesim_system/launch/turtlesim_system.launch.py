from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution, TextSubstitution

def generate_launch_description():
    ld = LaunchDescription()
    

    turtlesim = Node(
        package="turtlesim",
        executable="turtlesim_node",
        prefix = 'xterm -e',
        output='screen',
        name="turtlesim",
        remappings=[
          ("cmd_vel", "cmd_publisher")]
    )
    key_teleop = Node(
        package="turtlesim",
        executable="turtle_teleop_key",
        prefix = 'xterm -e',
        output='screen',
        name="key_teleop",
        remappings=[
          ("cmd_vel", "cmd_publisher")]
    )

    ld.add_action(turtlesim)
    ld.add_action(key_teleop)

    return ld

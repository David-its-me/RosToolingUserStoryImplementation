from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution, TextSubstitution

def generate_launch_description():
    ld = LaunchDescription()
    

    YourScript = Node(
        package="moveit2_scripts",
        executable="pick_and_place",
        prefix = 'xterm -e',
        output='screen',
        name="YourScript"
    )
    include_ur5e_cell_moveit_config= IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('ur5e_cell_moveit_config') + '/launch/robot.launch.py'])
    )

    ld.add_action(YourScript)
    ld.add_action(include_ur5e_cell_moveit_config)

    return ld

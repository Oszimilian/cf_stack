from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    input_grid_path_arg = DeclareLaunchArgument(
        'input_grid_path',
        default_value='default.json',
        description='path to the json-file which describs the grid of the arena'
    )

    input_grid_path = LaunchConfiguration('input_grid_path')

    path_planing = Node(
        package='cf_grid',
        executable='grid',
        output='screen',
        parameters=[{
            'input_grid_path': input_grid_path
        }]
    )


    return LaunchDescription(
        [
            input_grid_path_arg,
            path_planing
        ]
    )
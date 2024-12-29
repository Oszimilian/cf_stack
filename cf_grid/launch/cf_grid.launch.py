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

    x_segment_size_arg = DeclareLaunchArgument(
        'x_segment_size',
        default_value='0.2',
        description='x size of a segment'
    )

    y_segment_size_arg = DeclareLaunchArgument(
        'y_segment_size',
        default_value='0.2',
        description='y size of a segment'
    )

    x_size_offset_arg = DeclareLaunchArgument(
        'x_size_offset',
        default_value='0.2',
        description='grid beginns after x size offset'
    )

    y_size_offset_arg = DeclareLaunchArgument(
        'y_size_offset',
        default_value='0.2',
        description='grid beginns after y size offset'
    )

    input_grid_path = LaunchConfiguration('input_grid_path')
    x_segment_size = LaunchConfiguration('x_segment_size')
    y_segment_size = LaunchConfiguration('y_segment_size')
    x_size_offset = LaunchConfiguration('x_size_offset')
    y_size_offset = LaunchConfiguration('y_size_offset')

    grid = Node(
        package='cf_grid',
        executable='grid',
        output='screen',
        parameters=[{
            'input_grid_path' : input_grid_path,
            'x_segment_size' : x_segment_size,
            'y_segment_size' : y_segment_size,
            'x_size_offset' : x_size_offset,
            'y_size_offset' : y_size_offset
        }]
    )


    return LaunchDescription(
        [
            input_grid_path_arg,
            x_segment_size_arg,
            y_segment_size_arg,
            x_size_offset_arg,
            y_size_offset_arg,
            grid
        ]
    )